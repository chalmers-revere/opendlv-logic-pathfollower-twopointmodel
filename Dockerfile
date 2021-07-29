# Copyright (C) 2021 Bjornborg Nguyen
# Copyright (C) 2019 Ola Benderius
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

FROM alpine:latest as builder
RUN apk update && \
    apk --no-cache add \
    cmake \
    g++ \
    gcc \
    make \
    linux-headers
ADD . /opt/sources
WORKDIR /opt/sources
RUN mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/tmp .. && \
    make && make install


FROM alpine:latest

WORKDIR /usr/bin
COPY --from=builder /tmp/bin/opendlv-logic-pathfollower-twopointmodel .
ENTRYPOINT ["/usr/bin/opendlv-logic-pathfollower-twopointmodel"]
