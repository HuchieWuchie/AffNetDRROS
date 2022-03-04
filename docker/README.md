

# ROB9 docker images

This folder contains all the docker image(s) produced for the ROB9 semester project

## Pre-requisite

1. Pull the latest version of rob9 from github

2. Make sure you have docker and nvidia-docker installed

3. Navigate to this folder
```
cd ROB9ROOTDIR/docker
```

4. To visualize from docker images
```
xhost +
```

## GraspNet

This package is a docker image of the GraspNet from the implementation found here: https://github.com/graspnet/graspnet-baseline

```
@inproceedings{fang2020graspnet,
  title={GraspNet-1Billion: A Large-Scale Benchmark for General Object Grasping},
  author={Fang, Hao-Shu and Wang, Chenxi and Gou, Minghao and Lu, Cewu},
  booktitle={Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition(CVPR)},
  pages={11444--11453},
  year={2020}
}
```

### A. Use prebuilt docker image
```
docker pull huchiewuchie/graspnet-rob9
```

### B. Build the docker image

1. Build the graspnet base image containing the graspnet implementation

```
docker build --build-arg INCUBATOR_VER=$(date +%Y%m%d-%H%M%S) -t huchiewuchie/graspnet-base graspnet/graspnet-base

```

2. Build the docker image with the ROB9 files

```
docker build --build-arg INCUBATOR_VER=$(date +%Y%m%d-%H%M%S) -t huchiewuchie/graspnet-rob9 -f graspnet/graspnet-rob9/dockerfile ../
```

### C. Run the docker image

```
docker run --name graspnet-ros -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix --net=host --gpus all --rm huchiewuchie/graspnet-rob9 /bin/bash
```


## Affordance-Net

This package is a docker image of the affordancenet from the implementation found here: https://github.com/nqanh/affordance-net

```
@inproceedings{AffordanceNet18,
  title={AffordanceNet: An End-to-End Deep Learning Approach for Object Affordance Detection},
  author={Do, Thanh-Toan and Nguyen, Anh and Reid, Ian},
  booktitle={International Conference on Robotics and Automation (ICRA)},
  year={2018}
}
```

### A. Use prebuilt docker image

1. Pull docker image

```
docker pull huchiewuchie/affordancenet-rob9
```

### B. Build the docker image

1.
```
cd ROB9ROOTDIR/docker/affordancenet/affordancenet-base
```

2. Download the pre-trained weights from
here: https://drive.google.com/file/d/0Bx3H_TbKFPCjNlMtSGJlQ0dxVzQ/view?resourcekey=0-u8RCSHp2JpF9KJvj61lT6w
and save it to: ROB9ROOTDIR/docker/affordancenet/affordancenet-base/AffordanceNet_200K.caffemodel

3. Build affordancenet-base containing the implementation
```
docker build -t huchiewuchie/affordancenet-base affordancenet/affordancenet-base
```

4. build affordancenet-rob9 docker image with the ROB9 files
```
docker build --build-arg INCUBATOR_VER=$(date +%Y%m%d-%H%M%S) -t huchiewuchie/affordancenet-rob9 -f affordancenet/affordancenet-rob9/dockerfile ../
```

### C. Run the docker image

```
docker run --name affordancenet -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix --net=host --gpus all --rm huchiewuchie/affordancenet-rob9 /bin/bash
```

## Affordance-Net-Context

This package is a docker image of the affordancenet from the implementation found here: https://github.com/ivalab/affordanceNet_Context by Fu-Jen Chu, Ruinian Xu, Chao Tangand Patricio A. Vela.

### B. Build the docker image

1.
```
cd ROB9ROOTDIR/docker/affordancenet_context/affordancenet_context-base
```

2. Download the pre-trained weights from
here: https://www.dropbox.com/s/4wai7v9j6jp7pge/vgg16_faster_rcnn_iter_110000_pam_7attribute.caffemodel?dl=0
and save it to: ROB9ROOTDIR/docker/affordancenet_context/affordancenet_context-base/vgg16_faster_rcnn_iter_110000_pam_7attribute.caffemodel

3. Change directory back to docker base folder
```
cd ROB9ROOTDIR/docker/
```

4. Build affordancenet_context-base containing the implementation
```
docker build --build-arg INCUBATOR_VER=$(date +%Y%m%d-%H%M%S) -t huchiewuchie/affordancenet_context-base affordancenet_context/affordancenet_context-base
```

5. build affordancenet_context-rob9 docker image with the ROB9 files
```
docker build --build-arg INCUBATOR_VER=$(date +%Y%m%d-%H%M%S) -t huchiewuchie/affordancenet_context-rob9 -f affordancenet_context/affordancenet_context-rob9/dockerfile ../
```

### C. Run the docker image

```
docker run --name affordancenetcontext-rob9 -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix --net=host --gpus all --rm huchiewuchie/affordancenet_context-rob9 /bin/bash
```
