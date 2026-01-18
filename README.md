# Immersive VR showcase of the Megabot

**Description:** Turn the megabot into a VR simulation rig.

![image](./assets/Megabot.gif)

[👨‍💻 Developer documentation](docs/developer) • [📈 Project report](docs/report) • [📚 Bibliography](docs/bibliography)

## 📄 This project in short
This project's goal is to implement a unique way of showcasing the megabot mechanical capabilities by turning it into a motion simulation rig. More specifically, it uses the megabot to emulate the movement felt by the user during a rollercoaster ride.

The VR simulation is done by Epic Roller Coaster on a Quest 2 VR headset leaving the integrated computer of the megabot free to gather the telemetric data broadcasted through the local area network. It then generates the corresponding movement for the central platform using a classical washout algorithm tuned for the megabot. The movement is then executed by the already existing megabot control program, which the project is based on and requires to run

## 🚀 Quickstart
This project requires access to the megabot original codebase written by Rhoban.


### Install Instruction from this repository on the original megabot code
* Clone the megabot control repository ```git clone https://github.com/Rhoban/megabot.git```
* Go to the root of the newly cloned repository ```cd ./megabot```
* Clone this repository ```git clone <TO BE COMPLETED> ```
* Install poetry with pipy ```pip install poetry```
* Install dependencies using poetry ```poetry install```

You now have all that is required to run the program. Altough in order to run the project through the UI of the megabot, an option need to be added in the UI menu to launch the *RollerCoasterLinear* or *RollerCoasterWashout* task declared in [rollercoaster_task.py](./src/rollercoaster_task.py). The option should be added in the *full.py* script in the original megabot code.

### Install Instruction from the preinstalled branch
* Clone the megabot control repository ```git clone https://github.com/Rhoban/megabot.git```
* Change branch to RollerCoasterMain ```git branch RollerCoasterMain```

### Launch instructions:
* With poetry : ```poetry run python ./full.py```
* Without poetry :  ```python ./full.py```
* Turn on the VR headset and boot up Epic Roller Coaster
* Go into the option menu and look for the "Hardware" submenu
* At the bottom of the page, write the local area network ipv4 address of the megabot or your test device
* Launch the broadcast with the button below the address input field
* On the megabot control menu, press "Dance setup (40cm)" then "RollerCoaster(Linéaire)" or "RollerCoaster(Washout)"
* Launch any track on Epic Roller Coaster

Now the megabot should follow along the movement of the game.

### Switching between real and fake telemetry data
To test the megabot's response without running the VR simulation, a fake telemetry reader can be enabled. When the task [RollerCoaster](./docs/developer/README.md) is created a bool is defined, ```True``` for fake telemetry or ```False``` for the real telemetry.

## 🔍 About this project

|       |        |
|:----------------------------:|:-----------------------------------------------:|
| 💼 **Client**                |  Eirlab                                         |
| 🔒 **Confidentiality**       | **Public**                                      |
| ⚖️ **License**               |  [GNU GENERAL PUBLIC LICENSE](./LICENSE)*       |
| 👨‍👨‍👦 **Authors**               |  Nicolas Cornet, Antonin De Bouter              |
| 📧 **E-mails**               |  nicolas.cornet@ensc.fr, adebouter@enseirb-matmeca.fr |
