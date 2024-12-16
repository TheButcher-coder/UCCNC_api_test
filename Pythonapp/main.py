import streamlit as st
import serial
import time
import subprocess

st.title("Spotter")

# Initialize session state variables
if 'exe' not in st.session_state:
    st.session_state.exe = ''
if 'coom' not in st.session_state:
    st.session_state.coom = ''
if 'rate' not in st.session_state:
    st.session_state.rate = ''
if 'dwell' not in st.session_state:
    st.session_state.dwell = ''
if 'gpath' not in st.session_state:
    st.session_state.gpath = ''
if 'dest' not in st.session_state:
    st.session_state.dest = ''

# Input fields for user input
st.session_state.exe = st.text_input('Path of executable:', st.session_state.exe)
st.session_state.coom = st.text_input('Number of COM port:', st.session_state.coom)
st.session_state.rate = st.text_input('Spot rate:', st.session_state.rate)
st.session_state.dwell = st.text_input('Dwell time:', st.session_state.dwell)
st.session_state.gpath = st.text_input('Path of gfile:', st.session_state.gpath)
st.session_state.dest = st.text_input('Path of outputs:', st.session_state.dest)

# DEV quick start logic
if st.button("DEV quick start"):
    st.session_state.exe = "E:\\Desktop\\Arbeit\\App\\pythonProject\\src\\gesamt.exe"
    st.session_state.rate = '1'
    st.session_state.dwell = '0'
    st.session_state.gpath = "E:\\Desktop\\Arbeit\\App\\pythonProject\\src\\snaek.txt"
    st.session_state.dest = "E:\\Desktop\\Arbeit\\App\\pythonProject\\src\\out"
    st.session_state.coom = '15'
    st.session_state.isdev = True
else:
    st.session_state.isdev = False

# Main start logic
if st.button('Start'):
    if (st.session_state.rate and st.session_state.dwell and st.session_state.gpath and st.session_state.dest) or st.session_state.isdev:
        # Activate Arduino Connection
        ser = serial.Serial('COM' + st.session_state.coom, 9600, timeout=1)
        time.sleep(1)
        ser.close()

        # Execute subprocess only when all fields are filled
        result = subprocess.run(
            [
                st.session_state.exe,
                st.session_state.rate,
                st.session_state.dwell,
                "abc",
                st.session_state.gpath,
                st.session_state.dest,
                st.session_state.coom
            ],
            capture_output=True,
            text=True
        )

        # Display the result
        st.write("Result:", result.stdout)
        st.write("Error (if any):", result.stderr)

        # Stop button to stop the app
        if st.button('Stop'):
            st.stop()
    else:
        st.warning("Please fill in all fields before starting.")

st.stop()
