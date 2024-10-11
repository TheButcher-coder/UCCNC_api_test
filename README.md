# UCCNC API Controller to sync an HPLC to a stepcraft D420 with UC100 Controller


Steps to use it:

(UCCNC has to be installed in the Path c:\UCCNC else it wont work this way)
1. Launch Arduino IDE to Upload the given Code to an Arduino Nano thats wired as given below
2. Check that the Arduino is writing data to the Serial Port (THIS STEP HAS TO BE DONE EVERY TIME, else nothing will be read in the c++ Program)
3. Close Arduino IDE
4. Launch the python script main.py by using: streamlit run main.py
5. put in all the necessary paths
6. press 'start'

(if the UCCNC install Folder is elsewhere go to main.cpp and change the Path in line 37)
![schem - page 1](https://github.com/user-attachments/assets/088549bb-6fda-4c80-af8f-43a9773c1b24)
