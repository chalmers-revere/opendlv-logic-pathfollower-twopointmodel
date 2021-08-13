/*
 * Copyright (C) 2021 Björnborg Nguyen
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

#include "CvPlot/cvplot.h"

int32_t main(int32_t argc, char **argv)
{
  int32_t retCode{1};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if ((0 == commandlineArguments.count("cid")) ||
      (0 == commandlineArguments.count("freq")) ||
      (0 == commandlineArguments.count("rec-path")) ||
      (0 == commandlineArguments.count("max-preview-distance")) ||
      (0 == commandlineArguments.count("lateral-error-gain")) ||
      (0 == commandlineArguments.count("time-to-align"))
      // (0 == commandlineArguments.count("time-to-arrive"))
  )
  {
    std::cerr << argv[0] << " follows a GPS path from the given .rec file by"
              << "using the Two point model of steering (Salvucci and Gray, 2004) combined with a PID speed controller."
              << std::endl
              << "Usage:   " << argv[0] << " --cid=<CID> --freq=<Frequency to send> "
              << "--rec-path=<File where the GPS path is stored> "
              << "--max-preview-distance=<Max preview distance> "
              << "--time-to-align=<Time-to-align the preview point in yaw angle> "
              << "--time-to-arrive=<Time-to-arrive to preview point>"
              << "[--id-input=<Sender stamp of GPS input>]"
              << "[--id-output=<Sender stamp of motion request output>]"
              << "[--speedtarget=<target constant speed>]"
              << "[--verbose]"
              << std::endl
              << "Example: " << argv[0] << " --cid=111 --freq=20 "
              << "--rec-path=gps-path.rec --max-preview-distance=20.0 "
              << "--time-to-align=2.5" << std::endl;
  }
  else
  {
    // 2021-08-10 17:06:05 | Curvature, adaptive speed?
    // 2021-08-10 17:06:23 | Hack next gnss trace?
    // 2021-08-10 17:06:56 | Optimal speed?

    // 2021-08-10 14:55:13 | 50 km /h, steerincoeff 10, maxpreview 30,time2align 2.7
    // 2021-08-10 14:55:13 | 70 km /h, steerincoeff 16.8, maxpreview 25, time2align 2.2

    // 2021-08-13 12:52:25 | 50 km/h ,steerincoeff 16.8, maxpreview 15, time2align 2.0
    // 2021-08-13 13:51:31 | 60 km/h ,steerincoeff 16.8, maxpreview 18, time2align 2.0
    // 2021-08-13 14:09:56 | 70 km/h ,steerincoeff 16.8, maxpreview 21, time2align 2.0

    // 2021-08-13 14:34:09 | Higher speed gives more different vehicle dynamics, in regards to steering. Slip angle makes it hard to steer. One potential fix is to increase steering coef.

    // 2021-08-13 15:36:29 | Error sign in lateral-error-gain for y;

    uint32_t const senderStampInput{
        (commandlineArguments.count("id-input") != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["id-input"])) : 0};
    uint32_t const senderStampOutput{
        (commandlineArguments.count("id-output") != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["id-output"])) : 0};
    bool const verbose{commandlineArguments.count("verbose") != 0};

    // 2021-08-09 21:40:52 | What does these do?
    float maxPreviewDistance{
        std::stof(commandlineArguments["max-preview-distance"])};
    float timeToAlign{std::stof(commandlineArguments["time-to-align"])};
    float lateralErrorGain{std::stof(commandlineArguments["lateral-error-gain"])};
    // float timeToArrive{std::stof(commandlineArguments["time-to-arrive"])};
    double const constantSpeedTarget{(commandlineArguments.count("speedtarget") != 0) ? std::stod(commandlineArguments["speedtarget"]) : 5.0 / 3.6};

    cluon::OD4Session od4{
        static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

    std::vector<std::array<double, 2>> globalPath;
    bool globalPathIsClosed{false};
    uint32_t maxInd{0};

    {
      double maxDistance{0.0};
      cluon::Player player(commandlineArguments["rec-path"], false, false);
      while (player.hasMoreData())
      {
        auto next = player.getNextEnvelopeToBeReplayed();
        if (next.first)
        {
          cluon::data::Envelope env{std::move(next.second)};
          if (env.dataType() == opendlv::proxy::GeodeticWgs84Reading::ID())
          {
            auto msg =
                cluon::extractMessage<opendlv::proxy::GeodeticWgs84Reading>(
                    std::move(env));

            // od4.send(msg, cluon::time::now(), 99);

            std::array<double, 2> pos{msg.latitude(), msg.longitude()};
            if (!globalPath.empty())
            {
              auto prevPos{globalPath.back()};
              auto c = wgs84::toCartesian(prevPos, pos);
              double distance{sqrt(c[0] * c[0] + c[1] * c[1])};
              maxInd = (distance > maxDistance) ? globalPath.size() : maxInd;
              maxDistance = (distance > maxDistance) ? distance : maxDistance;
            }
            globalPath.push_back(pos);
          }
        }
      }
      if (globalPath.size() < 1)
      {
        std::cerr << "No global path was found in the give .rec file."
                  << std::endl;
        return 1;
      }
      auto c = wgs84::toCartesian(globalPath.front(), globalPath.back());
      double distance{sqrt(c[0] * c[0] + c[1] * c[1])};
      // 2021-08-12 15:06:54 | Hard condition to meet (distance < maxDistance) which is rouhgly 0.5m.
      // Changing this to a hardcoded constant, 5m.
      if (distance < 5.0)
      {
        globalPathIsClosed = true;
      }
      if (verbose)
      {
        std::cout << "The global path was loaded, it contains "
                  << globalPath.size() << " points with the longest distance "
                  << maxDistance << " m between points at ind " << maxInd << ".";
        if (globalPathIsClosed)
        {
          std::cout << " The path is closed.";
        }
        std::cout << std::endl;
      }
    }

    bool hasPrevPos = false;
    std::mutex wgsMutex;
    std::array<double, 2> curPos{globalPath.front()};
    std::array<double, 2> curAimpoint{globalPath.front()};
    std::array<double, 2> prevPos{};
    int32_t closestGlobalPointIndex{-1};

    auto onGeodeticWgs84Reading{
        [&od4, &wgsMutex, &curPos, &curAimpoint, &prevPos, &hasPrevPos, &globalPath,
         &closestGlobalPointIndex, &globalPathIsClosed, &maxPreviewDistance,
         &timeToAlign, &lateralErrorGain,
         //  &timeToArrive,
         &senderStampInput, &senderStampOutput,
         &constantSpeedTarget, &verbose](cluon::data::Envelope &&envelope)
        {
          if (envelope.senderStamp() == senderStampInput)
          {
            auto msg =
                cluon::extractMessage<opendlv::proxy::GeodeticWgs84Reading>(
                    std::move(envelope));
            std::array<double, 2> pos{msg.latitude(), msg.longitude()};

            // Step 1: Find heading based on two positions
            if (!hasPrevPos)
            {
              prevPos = pos;
              hasPrevPos = true;
              return;
            }
            // 2021-08-10 09:40:59 | This is causing inaccurate estimation at slow speed, especially stand still
            double heading;
            {
              std::array<double, 2> direction = wgs84::toCartesian(prevPos, pos);
              heading = atan2(direction[1], direction[0]);
            }

            // Step 2: Find global point closest to the current position
            int32_t closestPointIndex = -1;
            double minDistance = std::numeric_limits<double>::max();
            double prevDistanceLo = std::numeric_limits<double>::max();
            double prevDistanceHi = std::numeric_limits<double>::max();
            for (uint32_t i = 0; i <= globalPath.size() / 2; ++i)
            {
              bool isIncreasingDistanceLo;
              {
                int32_t j0 = closestGlobalPointIndex - i;
                j0 = (j0 < 0) ? globalPath.size() + j0 : j0;
                auto posLo{globalPath[j0]};

                auto c = wgs84::toCartesian(pos, posLo);
                double distance{sqrt(c[0] * c[0] + c[1] * c[1])};
                if (distance < minDistance)
                {
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
                         ? j1 - globalPath.size()
                         : j1;
                auto posHi{globalPath[j1]};

                auto c = wgs84::toCartesian(pos, posHi);
                double distance{sqrt(c[0] * c[0] + c[1] * c[1])};
                if (distance < minDistance)
                {
                  minDistance = distance;
                  closestPointIndex = j1;
                }
                isIncreasingDistanceHi = (distance > prevDistanceHi);
                prevDistanceHi = distance;
              }
              if (closestGlobalPointIndex != -1 && isIncreasingDistanceLo && isIncreasingDistanceHi)
              {
                break;
              }
            }
            closestGlobalPointIndex = closestPointIndex;
            prevPos = pos;

            // Step 3: Find what direction to go, based on heading
            // 2021-08-09 16:55:35 | This is not currently used?
            // double j0Heading;
            // double j1Heading;
            // {
            //   int32_t j0 = closestPointIndex - 1;
            //   if (j0 == -1)
            //   {
            //     j0 = globalPathIsClosed ? globalPath.size() - 1 : 0;
            //   }
            //   std::array<double, 2> j0Direction = wgs84::toCartesian(prevPos,
            //                                                          globalPath[j0]);
            //   j0Heading = atan2(j0Direction[1], j0Direction[0]);
            // }
            // {
            //   int32_t j1 = closestPointIndex + 1;
            //   if (j1 == static_cast<int32_t>(globalPath.size()))
            //   {
            //     j1 = globalPathIsClosed ? 0 : globalPath.size() - 1;
            //   }
            //   std::array<double, 2> j1Direction = wgs84::toCartesian(prevPos,
            //                                                          globalPath[j1]);
            //   j1Heading = atan2(j1Direction[1], j1Direction[0]);
            // }

            bool goingBackwards = false;
            //            (fabs(heading - j0Heading) < fabs(heading - j1Heading));

            // Step 4: Find aim point
            std::array<double, 2> aimPoint = globalPath[closestPointIndex];
            double aimPointAngle{};
            double aimPointDistance{};
            for (int32_t i = 1;; ++i)
            {
              int32_t j;
              if (goingBackwards)
              {
                j = closestPointIndex - i;
                if (j < 0)
                {
                  j = globalPathIsClosed ? globalPath.size() - i : 0;
                }
              }
              else
              {
                j = closestPointIndex + i;
                if (j > static_cast<int32_t>(globalPath.size()) - 1)
                {
                  j = globalPathIsClosed ? j - globalPath.size()
                                         : globalPath.size() - 1;
                }
              }

              double distance;
              {
                auto c = wgs84::toCartesian(aimPoint, globalPath[j]);
                distance = aimPointDistance + sqrt(c[0] * c[0] + c[1] * c[1]);
                if (distance > maxPreviewDistance)
                {
                  break;
                }
              }

              double angle;
              {
                double const pi = 3.1415926535;
                auto direction = wgs84::toCartesian(pos, globalPath[j]);
                angle = atan2(direction[1], direction[0]) - heading;
                while (angle < -pi)
                {
                  angle += 2.0 * pi;
                }
                while (angle > pi)
                {
                  angle -= 2.0 * pi;
                }
              }

              aimPoint = globalPath[j];
              aimPointAngle = angle;
              aimPointDistance = distance;

              {
                if (!globalPathIsClosed && ((goingBackwards && j == 0) || (!goingBackwards && j == static_cast<int32_t>(globalPath.size()) - 1)))
                {
                  if (verbose)
                  {
                    std::cout << "Reached the end of the global path."
                              << std::endl;
                  }
                  break;
                }
              }
            }

            // 2021-08-09 17:07:40 | This is probably some debugging information. Disabled for now.
            // Current aimpoint taken from pre-recorded gnss trace
            // opendlv::proxy::GeodeticWgs84Reading wgs84;
            // wgs84.latitude(aimPoint[0]);
            // wgs84.longitude(aimPoint[1]);
            // od4.send(wgs84, cluon::time::now(), 98);

            // Step 4.5: Find lateral error

            double lateralError;
            {
              auto p0 = wgs84::toCartesian(pos, aimPoint);
              auto p1 = wgs84::toCartesian(pos, globalPath[closestPointIndex]);

              double previewAngle = atan2(p0[1], p0[0]);

              // counter rotation
              //double errX = p1[0] * cos(-previewAngle) - p1[1] * sin(-previewAngle);
              double errY = p1[0] * sin(-previewAngle) + p1[1] * cos(-previewAngle);

              lateralError = errY;
              if (verbose)
              {
                std::cout << "Lateral error: " << lateralError << std::endl;
              }
            }

            // Step 5: Calculate and send control
            double vx = constantSpeedTarget;
            double yawRate = timeToAlign * aimPointAngle + lateralErrorGain * lateralError;

            if (verbose)
            {
              // std::cout << "aimPointAngle: " << aimPointAngle << " aimPointDistance: " << aimPointDistance << std::endl;

              std::cout << "Sends vx: " << vx << " yaw rate: " << yawRate << std::endl;
            }

            if (std::abs(lateralError) > 0.3)
            {
              yawRate += lateralErrorGain * lateralError;
              if (verbose)
              {
                // std::cout << "aimPointAngle: " << aimPointAngle << " aimPointDistance: " << aimPointDistance << std::endl;

                std::cout << "Modified yaw rate: " << yawRate << std::endl;
              }
            }

            opendlv::proxy::GroundMotionRequest gmr;
            gmr.vx(static_cast<float>(vx));
            gmr.yawRate(static_cast<float>(yawRate));

            od4.send(gmr, cluon::time::now(), senderStampOutput);
            {
              std::lock_guard<std::mutex> l(wgsMutex);
              curPos = pos;
              curAimpoint = aimPoint;
            }
          }
        }};

    od4.dataTrigger(opendlv::proxy::GeodeticWgs84Reading::ID(),
                    onGeodeticWgs84Reading);

    auto refCanvas{CvPlot::makePlotAxes()};

    auto refCanvasInit{
        [&refCanvas, &globalPath, &maxInd]()
        {
          std::vector<double> xCartesianCord;
          std::vector<double> yCartesianCord;
          std::array<double, 2> reference = globalPath.front();
          for (std::array<double, 2> wgs84Point : globalPath)
          {
            auto cartesianCords = wgs84::toCartesian(reference, wgs84Point);
            xCartesianCord.push_back(cartesianCords.front());
            yCartesianCord.push_back(cartesianCords.back());
          }
          refCanvas.create<CvPlot::Series>(xCartesianCord, yCartesianCord, "b.").setName("Preloaded GNSS path");
          // cvCanvas.create<CvPlot::Series>(std::vector<double>{xCartesianCord.front()}, std::vector<double>{yCartesianCord.front()}, "go").setName("Start");
          // cvCanvas.create<CvPlot::Series>(std::vector<double>{xCartesianCord.back()}, std::vector<double>{yCartesianCord.back()}, "ro").setName("End");
          // 10081
          refCanvas.create<CvPlot::Series>(std::vector<double>{xCartesianCord.at(maxInd)}, std::vector<double>{yCartesianCord.at(maxInd)}, "ro").setName("Largest jump");

          // cvCanvas.create<CvPlot::Series>(std::vector<double>{xCartesianCord.at(maxInd)}, std::vector<double>{yCartesianCord.at(maxInd)}, "go").setName("Current");
          // cvCanvas.create<CvPlot::Series>(std::vector<double>{xCartesianCord.at(maxInd)}, std::vector<double>{yCartesianCord.at(maxInd)}, "r.").setName("Aimpoint");

          refCanvas.title("Map");
          refCanvas.xLabel("x [m]");
          refCanvas.yLabel("y [m]");
          refCanvas.create<CvPlot::Legend>().setParentAxes(&refCanvas);

          // CvPlot::show("Path", plotCanvas);
          // auto PlotImage{plotCanvas.render(800, 800).getUMat(cv::ACCESS_READ)};
          // std::cout << "Plotting the loaded path." << std::endl;
          // cv::imshow("Plotting", PlotImage);
          // cv::waitKey();

          cv::imshow("Loaded GNSS map", refCanvas.render(800, 800).getUMat(cv::ACCESS_READ));
          cv::waitKey(1000);
        }};
    refCanvasInit();

    auto realtimeCanvas{CvPlot::makePlotAxes()};

    auto initCanvas{
        [&realtimeCanvas, &wgsMutex, &curPos, &curAimpoint, &globalPath]()
        {
          std::array<double, 2> curPosCopy;
          std::array<double, 2> curAimpointCopy;
          {
            std::lock_guard<std::mutex> l(wgsMutex);
            curPosCopy = curPos;
            curAimpointCopy = curAimpoint;
          }
          // Generate relative map
          std::vector<double> xCartesianCord;
          std::vector<double> yCartesianCord;
          // std::array<double, 2> reference = globalPath.front();
          for (std::array<double, 2> wgs84Point : globalPath)
          {
            auto cartesianCords = wgs84::toCartesian(curPosCopy, wgs84Point);
            xCartesianCord.push_back(cartesianCords.front());
            yCartesianCord.push_back(cartesianCords.back());
          }
          realtimeCanvas.create<CvPlot::Series>(xCartesianCord, yCartesianCord, "b.").setName("Preloaded GNSS path");

          // Current position
          realtimeCanvas.create<CvPlot::Series>(std::vector<double>{0.0}, std::vector<double>{0.0}, "ro").setName("Current pos");
          // Current aimpoint
          auto curAimpointXY = wgs84::toCartesian(curPosCopy, curAimpointCopy);
          realtimeCanvas.create<CvPlot::Series>(
                            std::vector<double>{0.0, curAimpointXY.front()},
                            std::vector<double>{0.0, curAimpointXY.back()},
                            "k-o")
              .setName("Current aimpoint")
              .setMarkerSize(4);

          realtimeCanvas.title("Map");
          realtimeCanvas.xLabel("x [m]");
          realtimeCanvas.yLabel("y [m]");
          uint8_t lim{30};
          realtimeCanvas.setXLim({-lim, lim})
              .setYLim({-lim, lim})
              .setFixedAspectRatio();
          realtimeCanvas.create<CvPlot::Legend>().setParentAxes(&realtimeCanvas);
        }};

    auto renderCanvas{
        [&realtimeCanvas, &wgsMutex, &curPos, &curAimpoint, &globalPath]()
        {
          std::array<double, 2> curPosCopy;
          std::array<double, 2> curAimpointCopy;
          {
            std::lock_guard<std::mutex> l(wgsMutex);
            curPosCopy = curPos;
            curAimpointCopy = curAimpoint;
          }
          // Current aimpoint
          auto curAimpointXY = wgs84::toCartesian(curPosCopy, curAimpointCopy);

          // std::cout << "Current relative aim: " << curAimpointXY.front() << ", " << curAimpointXY.back() << std::endl;
          // realtimeCanvas.find<CvPlot::Series>("Current aimpoint")->setPoints(std::vector<cv::Point2d>{{curAimpointXY.front(), curAimpointXY.back()}});
          realtimeCanvas.find<CvPlot::Series>("Current aimpoint")->setX(std::vector<double>{0.0, curAimpointXY.front()});
          realtimeCanvas.find<CvPlot::Series>("Current aimpoint")->setY(std::vector<double>{0.0, curAimpointXY.back()});

          // realtimeCanvas.create<CvPlot::Series>(std::vector<double>{curAimpointXY.front()}, std::vector<double>{curAimpointXY.back()}, "b.").setName("Current aimpoint");

          // Generate relative map
          std::vector<double>
              xCartesianCord;
          std::vector<double> yCartesianCord;
          // std::array<double, 2> reference = globalPath.front();
          for (std::array<double, 2> wgs84Point : globalPath)
          {
            auto cartesianCords = wgs84::toCartesian(curPosCopy, wgs84Point);
            xCartesianCord.push_back(cartesianCords.front());
            yCartesianCord.push_back(cartesianCords.back());
          }
          // realtimeCanvas.create<CvPlot::Series>(xCartesianCord, yCartesianCord, "b.").setName("Preloaded GNSS path");
          realtimeCanvas.find<CvPlot::Series>("Preloaded GNSS path")->setX(xCartesianCord);
          realtimeCanvas.find<CvPlot::Series>("Preloaded GNSS path")->setY(yCartesianCord);

          cv::imshow("Debug window", realtimeCanvas.render(800, 800).getUMat(cv::ACCESS_READ));
        }};

    initCanvas();
    while (od4.isRunning())
    {
      renderCanvas();
      cv::waitKey(100);
      // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Reset control
    opendlv::proxy::GroundMotionRequest gmr;
    gmr.vx(0.0f);
    gmr.yawRate(0.0f);
    od4.send(gmr, cluon::time::now(), senderStampOutput);

    retCode = 0;
  }
  return retCode;
}
