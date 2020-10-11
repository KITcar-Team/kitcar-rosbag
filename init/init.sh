#!/bin/bash
# ask for init or cleanup
echo -e "\e[96mAttention this script installs packages (pip3)! \e[39m"
read -p "Do you want to initialize (1) or cleanup (2) kitcar-rosbag? " option

case "$option" in
  1) # do initialize
    # check for dependencies
    if [ -z "$KITCAR_REPO_PATH" ]; then
      echo -e "\n\e[31mERROR: can't find \$KITCAR_REPO_PATH"
      echo -e "\e[31mplease run kitcar-init script: https://git.kitcar-team.de/kitcar/kitcar-init first\e[39m"
      exit 1
    fi

    # initialize kitcar-gazebo-imulation
    echo -e "\nadd kitcar-rosbag bashrc to your bashrc"
    echo "source $KITCAR_REPO_PATH/kitcar-rosbag/init/bashrc # for kitcar-rosbag repository" >> ~/.bashrc

    # load changes
    echo "apply changes to current terminal ..."
    source  ~/.bashrc

    INIT_DIR=$KITCAR_REPO_PATH/kitcar-rosbag/init

    # Install python packages
    echo -e "\nStart installing python packages."
    pip3 install --upgrade --upgrade-strategy eager --no-warn-script-location -r $INIT_DIR/requirements.txt

    source ~/.profile

    # Install pre-commit hook
    cd $KITCAR_REPO_PATH/kitcar-rosbag
    pre-commit install
  ;;

  2) # do cleanup
    echo -e "\nremove kitcar-rosbag entry from bashrc"
    sed '/source .*kitcar-rosbag/d' -i ~/.bashrc

    echo "apply changes to current terminal ..."
    # Source init / This fails when the simulation has not been built
    source  ~/.bashrc
  ;;

  *) # invalid option
    echo -e "\n\e[33minvalid option, choose 1 or 2"
esac
