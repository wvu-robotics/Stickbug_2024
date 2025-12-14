#Incase of failure update Torch with following link: 
#https://developer.download.nvidia.com/compute/redist/jp/

local init_dir=$PWD
sudo apt-get install python3-pip
sudo apt-get install libopenblas-dev

export TORCH_INSTALL=https://developer.download.nvidia.com/compute/redist/jp/v60dp/pytorch/torch-2.2.0a0+81ea7a4.nv24.01-cp310-cp310-linux_aarch64.whl

python3 -m pip install --upgrade pip
python3 -m pip install numpy=='1.26.1'
python3 -m pip install --no-cache $TORCH_INSTALL

sudo apt-get install libjpeg-dev zlib1g-dev libpython3-dev libopenblas-dev libavcodec-dev libavformat-dev libswscale-dev
git clone --branch release/0.17 https://github.com/pytorch/vision /tmp/torchvision
export BUILD_VERSION=0.17.0
cd /tmp/torchvision
python3 setup.py install --user
trap "rm -rf /tmp/torchvision; cd $init_dir" 0

