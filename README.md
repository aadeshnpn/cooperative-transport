# Cooperative Transport
Social animals cooperatively transport objects which is many times bigger than themselves effectively. Mimicking those behaviors on real robots will have diverse applications in engineering, health care and search and rescue. In this work, we define different categories of cooperative transport problems and discuss different tools and techniques to tackle them. We then show that occlusion-based cooperative transport techniques are effective when the object is convex and there are enough agents to overcome frictional force. Results show that even with only two robots, the occlusion-based technique is able to transport objects 60\% of the time.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. 

### Prerequisites

What things you need to install the software and how to install them

#### Opencv
Use any tutorials on web to install opencv 3 on your machine. This demo requires opencv3 and opencv3-contrib modules.
https://gist.github.com/sbrugman/f9d897f28e674f7a89bbf131e26b98b0

#### Cozmo
Use the official documentation to install conzmo drivers and SDK on your machine from below
http://cozmosdk.anki.com/docs/install-linux.html

#### Cubes
For this demo to work, you will need two cubes which will act as goal and the object to transport. You need to tag both the objects with Acuro tags and register their dimensions in the ct.py script. 

### Installing

After opencv and cozmo is intalled on the machine, use the examples from the cozmo sdk to verify the installation is correct and cozmo can be acccesed from the machine.

## Demo

Afer the cozmo is functional, connect IOS/Android devices to the same machine. Then call the run.py script
```
python run.py
```
This scirpt will start async connection with both cozmo robot via the devices and they will start to perform cooperative transport. 

[![Cooperative Transport with Cozmo](https://i.ytimg.com/vi/WvW2b_O350E/hqdefault.jpg)](https://www.youtube.com/watch?v=WvW2b_O350E "Cooperative Transport")

## Paper
[Cooperative Transport in Mobile Robots](cooperative-transport-mobile.pdf)
## Contact

If you find this demo cool and would like to talk more about it, just send me an email :)


## Authors

* **Aadesh Neupane** - *Initial work* - [Aadeshnpn](https://github.com/aadeshnpn)

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* HCMI Lab
