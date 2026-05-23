# Raspberry Pi Camera Setup on Ubuntu

## Build rpicam-apps from source
```bash
sudo apt update
sudo apt install -y git meson ninja-build pkg-config \
  libboost-program-options-dev libdrm-dev libexif-dev \
  libjpeg-dev libtiff-dev libpng-dev libepoxy-dev

cd ~
git clone --depth=1 https://github.com/raspberrypi/rpicam-apps.git
cd rpicam-apps
meson setup build --prefix=/usr/local
meson configure build | grep -Ei 'av|ffmpeg|gstreamer|drm|qt'
ninja -C build
sudo ninja -C build install
sudo ldconfig
```

## List available cameras
```bash
LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH \
  /usr/local/bin/rpicam-hello --list-cameras
```

### Save image test
```bash
LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH \
LIBCAMERA_IPA_MODULE_PATH=/usr/local/lib/aarch64-linux-gnu/libcamera/ipa \
/usr/local/bin/rpicam-still -o ~/test.jpg --nopreview -t 1500
ls -lh ~/test.jpg
```

### Stream Test
```bash
LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH \
LIBCAMERA_IPA_MODULE_PATH=/usr/local/lib/aarch64-linux-gnu/libcamera/ipa \
/usr/local/bin/rpicam-vid -t 3000 --nopreview -o ~/test.h264
ls -lh ~/test.h264
```

### Maybe needed for memory
# keep only one vc4-kms-v3d line
sudo nano /boot/firmware/config.txt
dtoverlay=vc4-kms-v3d,cma-512

## Add user to video group
```bash
sudo usermod -aG video,render $USER
# log out completely, then log back in (or reboot)
```

### Retest
```bash
id -nG | tr ' ' '\n' | grep -E 'video|render'
LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH \
LIBCAMERA_IPA_MODULE_PATH=/usr/local/lib/aarch64-linux-gnu/libcamera/ipa \
/usr/local/bin/rpicam-still --immediate --nopreview --width 640 --height 480 --buffer-count 2 -o ~/test.jpg
ls -lh ~/test.jpg
```

## basic_cam_test.py