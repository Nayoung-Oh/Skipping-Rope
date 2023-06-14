# Skipping Rope Counting
2023 Spring CS565 Mini-Proejct

By Nayoung Oh and Jinseo Lee
## Prepare hardware
What you need to prepare
- Arduino Nano 33 BLE Sense
- Arduino Nano shiled
- 4 X AAA battery with a battery shiled
- One jump rope
- One 3D printing of the `modeling/jump_rope_handle.stl`
- One laser cutting of 3mm MDF of `modeling/lower_plate.dwg` and `modeling/upper_plate.dwg`
- Some supporters, bolts, and nuts (recommend to use nylon nut for connecting the rope handle)

Assemble the new handle and Arduino

<img src = "https://github.com/Nayoung-Oh/Skipping-Rope/blob/e91b26668ec77ed5c38910590827e0871f93ce0b/image/assembly.jpg" width="30%">

## Collect training data
You should upload `data_collection/data_collection.ino` into the board. This program will collect imu sensor data.

Then, using the `data_collection/data.py` to communicate with the arduino and save the data about 15 seconds. 

## Training your own model
Use the `Jump_Classifier_Training.ipynb` to train the model and convert into Arduino compatible format.

## Check the performance using K-Fold
Use the `Jump_Classifier_KFold.ipynb` to check the performance of the model using K-Fold

## Test in the real-world setting
Download the apk file `final_jump.apk` into your smartphone. Currently, only Android is supported.

Upload `final_program` into your Arduino.

And enjoy!
