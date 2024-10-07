import streamlit as st
import os
import signal
import subprocess

st.title("Spotter")

# Input fields for user input
exe = st.text_input('Path of executable:')
coom = st.text_input('Number of COM port:')
rate = st.text_input('Spot rate:')
dwell = st.text_input('Dwell time:')
gpath = st.text_input('Path of gfile:')
dest = st.text_input('Path of outputs:')

# Add a start button to trigger the process
if st.button('Start'):

    # Check if all fields have values
    if rate and dwell and gpath and dest:
        # Path to the executable
        path = exe

        # Execute subprocess only when all fields are filled
        result = subprocess.run([path, rate, dwell, "abc", gpath, dest, coom], capture_output=True, text=True)

        # Display the result
        st.write("Result:", result.stdout)
        st.write("Error (if any):", result.stderr)
    else:
        st.warning("Please fill in all fields before starting.")



st.stop()