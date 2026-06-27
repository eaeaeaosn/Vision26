sleep 10
cd ~/Vision26
# 最多等 60 秒，直到海康相机出现在 USB 上
for i in $(seq 1 60); do
    lsusb | grep -qi 2bdf && break
    sleep 1
done
mkdir -p logs
screen -S sp_vision -L -Logfile logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog -d -m bash -c "./watchdog.sh"