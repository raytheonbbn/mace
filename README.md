
[//]: # ( Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617. © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL) )

# Managing Academies Challenge Evaluation (MACE)
The MACE project includes the systems used to design, stage, monitor, and evaluate the Joint Services Academies Collaborative Autonomy Challenge. The Collaborative Autonomy Challenge is a simulated Combat Search and Rescue Scenario, developed in collaboration with the Joint Personnel Recovery Agency and is designed to enable cadets to focus on the autonomous collaboration of vehicles, using relatively simple hardware, with abstracted effects. The Challenge will develop future DoD leaders’ competencies in collaborative autonomous systems, identify and remove technical barriers related to sensors and actuation by focusing on collaborative autonomy, build lasting collaborations, and develop a practicum curriculum that reflects visions for future operating challenges. Over the course of the Challenge, cadets will design and develop small swarms of collaborative autonomous robotic platforms capable of conducting an abstracted combat search and rescue operation through prescribed interactions with artifacts placed throughout the physical exercise location. 


# Getting Started
To get started with MACE, it is recommended that Users begin by reading the *Introduction to MACE* document. This document provides a high-level overview of the MACE challenge and the terminology that will be used throughout the system. After reading *Introduction to MACE* document, participants of the MACE challenge should refer to the *User Manual* document. This document contains information regarding how to use the MACE system. Developers should start with the *Developer Manual* which provides a high-level overview regarding how to get started with contributing toward the MACE project. Each of these documents can be found within the `docs/manuals/` directory.
 
# MACE Installation
To install and run MACE for testing, please run the install_mace.bat script in your release directory. If you are working with the source code, please follow the instructions below instead.

# Running Docker for Target and Ros Sim
To run a docker container with a default of 6 targets and 2 rovers first build it by running:
docker build . --tag mace
Then run the docker container by running:
docker run -p 1883:1883 -p 5000:5000 -p 9091:9091 -p 9092:9092 -p 11312:11312 -p 11313:11313 -it mace

# Running the STOMP Sim UI
Navigate to the code/stomp directory and build the project by running
./gradlew jar shadowjar
Then run the STOMP Sim UI by running:
./start.sh
