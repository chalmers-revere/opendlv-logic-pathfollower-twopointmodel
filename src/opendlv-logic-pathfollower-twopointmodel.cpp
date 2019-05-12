/*
 * Copyright (C) 2019 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <cmath>
#include <iostream>
#include <string>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "WGS84toCartesian.hpp"

int32_t main(int32_t argc, char **argv)
{
  int32_t retCode{1};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if ( (0 == commandlineArguments.count("cid")) ||
       (0 == commandlineArguments.count("freq")) ||
       (0 == commandlineArguments.count("rec-path")) ) {
    std::cerr << argv[0] << " follows a GPS path from the given .rec file by"
      << "using the Two point model of steering (Salvucci and Gray, 2004)."
      << std::endl
      << "Usage:   " << argv[0] << " --cid=<CID> --freq=<Frequency to send> "
      << "--rec-path=<File where the GPS path is stored> "
      << "[--id-input=<Sender stamp of GPS input>]"
      << "[--id-output=<Sender stamp of motion request output>] [--verbose]" 
      << std::endl
      << "Example: " << argv[0] << " --cid=111 --freq=20 "
      << "--rec-path=gps-path.rec" << std::endl;
  } else {
    uint32_t const senderStampInput{
      (commandlineArguments.count("id-input") != 0) ?
        static_cast<uint32_t>(std::stoi(commandlineArguments["id-input"])) : 0};
    //uint32_t const senderStampOutput{
    //  (commandlineArguments.count("id-output") != 0) ?
    //    static_cast<uint32_t>(std::stoi(commandlineArguments["id-output"])) : 0};
    bool const verbose{commandlineArguments.count("verbose") != 0};

    cluon::OD4Session od4{
      static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

    std::vector<std::array<double, 2>> globalPath;
    bool globalPathIsClosed{false};

    {
      double maxDistance{0.0};
      cluon::Player player(commandlineArguments["rec-path"], false, false);
      while (player.hasMoreData()) {
        auto next = player.getNextEnvelopeToBeReplayed();
        if (next.first) {
          cluon::data::Envelope env{std::move(next.second)};
          if (env.dataType() == opendlv::proxy::GeodeticWgs84Reading::ID()) {
            auto msg =
              cluon::extractMessage<opendlv::proxy::GeodeticWgs84Reading>(
                  std::move(env));

            std::array<double, 2> pos{msg.latitude(), msg.longitude()};
            if (!globalPath.empty()) {
              auto prevPos{globalPath.back()};
              auto c = wgs84::toCartesian(prevPos, pos);
              double distance{sqrt(c[0] * c[0] + c[1] * c[1])};
              maxDistance = (distance > maxDistance) ? distance : maxDistance;
            }
            globalPath.push_back(pos);
          }
        }
      }
      if (globalPath.size() < 1) {
        std::cerr << "No global path was found in the give .rec file." 
          << std::endl;
        return 1;
      }
      auto c = wgs84::toCartesian(globalPath.front(), globalPath.back());
      double distance{sqrt(c[0] * c[0] + c[1] * c[1])};
      if (distance < maxDistance) {
        globalPathIsClosed = true;
      }
      if (verbose) {
        std::cout << "The global path was loaded, it contains " 
          << globalPath.size() << " points with the longest distance " 
          << maxDistance << " m between points.";
        if (globalPathIsClosed) {
          std::cout << " The path is closed.";
        }
        std::cout << std::endl;
      }
    }

    bool hasPrevPos = false;
    std::array<double, 2> prevPos{};
    int32_t closestGlobalPointIndex{-1};

    auto onGeodeticWgs84Reading{[&od4, &prevPos, &hasPrevPos, &globalPath, 
      &closestGlobalPointIndex, &globalPathIsClosed, &senderStampInput, 
      &verbose](cluon::data::Envelope &&envelope)
      {
        if (envelope.senderStamp() == senderStampInput) {
          auto msg = 
            cluon::extractMessage<opendlv::proxy::GeodeticWgs84Reading>(
                std::move(envelope));
          std::array<double, 2> pos{msg.latitude(), msg.longitude()};

          // Step 1: Find heading based on two positions
          if (!hasPrevPos) {
            prevPos = pos;
            hasPrevPos = true;
            return;
          }
          std::array<double, 2> direction = wgs84::toCartesian(prevPos, pos);
          double heading = atan2(direction[1], direction[0]);

          // Step 2: Find global point closest to the current position
          int32_t closestPointIndex = -1;
          double minDistance = std::numeric_limits<double>::max();
          double prevDistanceLo = std::numeric_limits<double>::max();
          double prevDistanceHi = std::numeric_limits<double>::max();
          for (uint32_t i = 0; i <= globalPath.size() / 2; ++i) {
            bool isIncreasingDistanceLo;
            {
              int32_t j0 = closestGlobalPointIndex - i;
              j0 = (j0 < 0) ? globalPath.size() + j0 : j0;
              auto posLo{globalPath[j0]};

              auto c = wgs84::toCartesian(pos, posLo);
              double distance{sqrt(c[0] * c[0] + c[1] * c[1])};
              if (distance < minDistance) {
                minDistance = distance;
                closestPointIndex = j0;
              }
              isIncreasingDistanceLo = (distance > prevDistanceLo);
              prevDistanceLo = distance;
            }
            bool isIncreasingDistanceHi;
            {
              int32_t j1 = closestGlobalPointIndex + i + 1;
              j1 = (j1 > static_cast<int32_t>(globalPath.size() - 1)) 
                ? j1 - globalPath.size() : j1;
              auto posHi{globalPath[j1]};

              auto c = wgs84::toCartesian(pos, posHi);
              double distance{sqrt(c[0] * c[0] + c[1] * c[1])};
              if (distance < minDistance) {
                minDistance = distance;
                closestPointIndex = j1;
              }
              isIncreasingDistanceHi = (distance > prevDistanceHi);
              prevDistanceHi = distance;
            }
            if (closestGlobalPointIndex != -1 && isIncreasingDistanceLo
                && isIncreasingDistanceHi) {
              break;
            }
          }
          closestGlobalPointIndex = closestPointIndex;
          prevPos = pos;

          // Step 3: Find what direction to go, based on heading
          std::vector<std::array<double, 2>> upcomingGlobalPath;
          upcomingGlobalPath.push_back(globalPath[closestPointIndex]);

          double j0Heading;
          double j1Heading;
          {
            int32_t j0 = closestPointIndex - 1;
            if (j0 == -1) {
              j0 = globalPathIsClosed ? globalPath.size() - 1 : 0;
            }
            std::array<double, 2> j0Direction = wgs84::toCartesian(prevPos,
                globalPath[j0]);
            j0Heading = atan2(j0Direction[1], j0Direction[0]);
          }
          {
            int32_t j1 = closestPointIndex + 1;
            if (j1 == static_cast<int32_t>(globalPath.size())) {
              j1 = globalPathIsClosed ? 0 : globalPath.size() - 1;
            }
            std::array<double, 2> j1Direction = wgs84::toCartesian(prevPos,
                globalPath[j1]);
            j1Heading = atan2(j1Direction[1], j1Direction[0]);
          }

          bool goingBackwards = 
            (fabs(heading - j0Heading) < fabs(heading - j1Heading));

          // Step 4: Extract upcoming global path
          double distanceToAdd = 100.0;
          for (int32_t i = 1; distanceToAdd > 0.0; ++i) {
            int32_t j;
            if (goingBackwards) {
              j = closestPointIndex - i;
              if (j < 0) {
                j = globalPathIsClosed ? globalPath.size() - i : 0;
              }
            } else {
              j = closestPointIndex + i;
              if (j > static_cast<int32_t>(globalPath.size()) - 1) {
                j = globalPathIsClosed ? j - globalPath.size() 
                  : globalPath.size() - 1;
              }
            }
            
            upcomingGlobalPath.push_back(globalPath[j]);
            auto c = wgs84::toCartesian(upcomingGlobalPath[i-1],
                upcomingGlobalPath[i]);
            distanceToAdd -= sqrt(c[0] * c[0] + c[1] * c[1]);
            if (!globalPathIsClosed && ((goingBackwards && j == 0) 
                || (!goingBackwards 
                  && j == static_cast<int32_t>(globalPath.size()) - 1))) {
              if (verbose) {
                std::cout << "Reached the end of the global path." 
                  << std::endl;
              }
              break;
            }
          }

          // Step 5: Transform into local frame



        }
      }};
    
    od4.dataTrigger(opendlv::proxy::GeodeticWgs84Reading::ID(),
        onGeodeticWgs84Reading);

    while (od4.isRunning()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    retCode = 0;
  }
  return retCode;
}

