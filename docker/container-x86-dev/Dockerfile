# DUA environment image.
#
# Roberto Masocco <r.masocco@dotxautomation.com>
#
# June 13, 2024

# Copyright 2024 dotX Automation s.r.l.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

FROM dotxautomation/dua-foundation:x86-dev

# Change this if you encounter problems with the default user
ARG USER_UID=1000

ENV DEBIAN_FRONTEND=noninteractive

# IMAGE SETUP START #
# go2_stack START #
# Configure Cyclone DDS RMW
RUN apt-get update && apt-get install -y --no-install-recommends \
  ros-humble-rosidl-generator-dds-idl && \
  rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*/apt/lists/*
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Install unitree_sdk2
WORKDIR /opt
RUN git clone --single-branch --branch 'main' --depth 1 https://github.com/unitreerobotics/unitree_sdk2.git && \
  cd unitree_sdk2 && \
  mkdir build && \
  cd build && \
  cmake -DCMAKE_BUILD_TYPE=RelWithDebugInfo -DBUILD_EXAMPLES=OFF .. && \
  make install && \
  cd ../.. && \
  chgrp -R internal /opt/unitree_sdk2 && \
  chmod -R g+rw /opt/unitree_sdk2
WORKDIR /root

# Install nlohmann/json C++ library
RUN mkdir -p /usr/local/include/nlohmann && \
  wget -O /usr/local/include/nlohmann/json.hpp https://github.com/nlohmann/json/releases/download/v3.11.3/json.hpp
# go2_stack END #
# IMAGE SETUP END #

# Create a user with Zsh as shell, hashed password, and add it to the sudoers
# To generate the hashed password, run:
#   mkpasswd -m sha-512 PASSWORD duatemplate
RUN useradd -r -m -s /usr/bin/zsh -u ${USER_UID} -G adm,dialout,internal,plugdev,sudo,tty,video -p '$6$duatemplate$LqsjNJBqytZVstpRVUT2telEwvGRNHirI3Fxx9vHMIfGBZSh1aqq/UytLVyaRtBylmehx00.3p3HQo4Gy8D6d1' neo
ENV HOME=/home/neo

# Create workspace directory: host workspaces will be mounted here
RUN mkdir ${HOME}/workspace && \
  chown neo:neo ${HOME}/workspace

# Create directory for shell history file
RUN mkdir ${HOME}/zsh_history && \
  chown neo:neo ${HOME}/zsh_history

# Create SSH directory for user
RUN mkdir ${HOME}/.ssh

# Switch to internal user
USER neo
WORKDIR ${HOME}

# Copy user configuration files
COPY --chown=neo:neo ./aliases.sh ./.aliases.sh
COPY --chown=neo:neo ./bashrc ./.bashrc
COPY --chown=neo:neo ./colcon-defaults.yaml /home/neo/.colcon/defaults.yaml
COPY --chown=neo:neo ./commands.sh ./.commands.sh
COPY --chown=neo:neo ./nanorc ./.nanorc
COPY --chown=neo:neo ./ros2.sh ./.ros2.sh
COPY --chown=neo:neo ./vimrc ./.vimrc

# Configure Zsh for internal user
ENV ZSH=${HOME}/.oh-my-zsh
ENV ZSH_CUSTOM=${ZSH}/custom
ENV ZSH_PLUGINS=${ZSH_CUSTOM}/plugins
ENV ZSH_THEMES=${ZSH_CUSTOM}/themes
RUN wget -qO- https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh | zsh || true
RUN git clone --single-branch --branch 'master' --depth 1 https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_PLUGINS}/zsh-syntax-highlighting \
  && git clone --single-branch --branch 'master' --depth 1 https://github.com/zsh-users/zsh-autosuggestions ${ZSH_PLUGINS}/zsh-autosuggestions \
  && git clone --single-branch --depth 1 https://github.com/romkatv/powerlevel10k.git ${ZSH_THEMES}/powerlevel10k
COPY --chown=neo:neo ./p10k.zsh ./.p10k.zsh
COPY --chown=neo:neo ./zshrc ./.zshrc

ENV DEBIAN_FRONTEND=dialog

# By default, start a basic shell
CMD ["bash"]
