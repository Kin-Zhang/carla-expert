#!/usr/bin/env bash
: '
Author: Kin (kin_eng@163.com)
Date: 2022-06-17
Usage: Download CARLA script!
!! 对应在中国大陆的同学 可以将函数换成 download_carla_mainland 即可 快速下载 mirror from SUSTech
Origin version from: https://github.com/autonomousvision/transfuser/blob/main/setup_carla.sh
'
function download_carla_mainland () {
    mkdir CARLA_0.9.$n
    cd CARLA_0.9.$n
    if ! wget https://mirrors.sustech.edu.cn/carla/carla/0.9.$n/CARLA_0.9.$n.tar.gz;
        then echo "There is no version for CARLA 0.9.$n";
    else
        tar -xvzf CARLA_0.9.$n.tar.gz
        echo "Finished download CARLA 0.9.$n... ==> now for additional maps"
        cd Import && wget https://mirrors.sustech.edu.cn/carla/carla/0.9.$n/AdditionalMaps_0.9.$n.tar.gz
        cd .. && bash ImportAssets.sh
        rm CARLA_0.9.$n.tar.gz Import/AdditionalMaps_0.9.$n.tar.gz
    fi
}

function download_carla () {
    mkdir CARLA_0.9.$n
    cd CARLA_0.9.$n
    if ! wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.$n.tar.gz;
        then echo "There is no version for CARLA 0.9.$n";
    else
        tar -xvzf CARLA_0.9.$n.tar.gz
        echo "Finished download CARLA 0.9.$n... ==> now for additional maps"
        cd Import && wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/AdditionalMaps_0.9.$n.tar.gz
        cd .. && bash ImportAssets.sh
        rm CARLA_0.9.$n.tar.gz Import/AdditionalMaps_0.9.$n.tar.gz
    fi
}

# echo "Whether Download CARLA? choice: [y, n]"
# read if_download
# if [ $if_download == 'y' ];
#     then
echo "Enter the CARLA version you need, choices: [10.1, 11, 12, 13]"
read n
echo "back to main home now ===> cd ~"
cd ~
$(download_carla)
# else
#     echo "skip for CARLA download"
# fi