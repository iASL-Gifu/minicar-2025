#  supervised train - Minicar 2025

## setup
```bash
python3 -m venv env
source env/bin/activate
pip3 install -r requirements.txt
```

## 1. extract data from rosbag file
```bash
python3 --bags_dir <input_bag_dir> --outdir ./datasets/extracted
```
## 2. train

## 3. convert pth to onnx
``` bash
python3 3_convert_weight_format.py -c <intput_path> -o ./outputs/pilot_net.onnx
```
## deploy model (on Jetson)
You must run this script on Jetson device. 
```bash
./deploy_model.sh -i pilotnet.onnx 
```
