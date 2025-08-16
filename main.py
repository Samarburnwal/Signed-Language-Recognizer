# import serial

# # Change the COM port and baud rate according to your Arduino setup
# ser = serial.Serial('COM10', 115200)  # For Linux/Mac, use '/dev/ttyUSB0' or similar

# print("Reading from serial... Press Ctrl+C to stop.\n")

# try:
#     while True:
#         if ser.in_waiting:
#             line = ser.readline().decode('utf-8').strip()
#             print(line)
# except KeyboardInterrupt:
#     print("\nStopped reading.")
#     ser.close()


# import serial
# import joblib
# import numpy as np
# import time

# # Load your trained model
# model = joblib.load('Signed_Lang_Model_New.pkl')

# # Open serial port (adjust COM port and baud rate)
# ser = serial.Serial('COM10', 115200)
# time.sleep(2)  # Allow connection to stabilize

# try:
#     while True:
#         if ser.in_waiting > 0:
#             line = ser.readline().decode('utf-8').strip()
            
#             # Assuming your Arduino sends comma-separated values like: 45,23,678,...
#             parts = line.split()
            
#             try:
#                 # Convert to float or int and reshape for model
#                 data = np.array([float(x) for x in parts]).reshape(1, -1)

#                 # Make prediction
#                 prediction = model.predict(data)
#                 print("Prediction:", prediction[0])

#             except ValueError:
#                 print("Received invalid data:", line)

# except KeyboardInterrupt:
#     print("Stopped by user.")
#     ser.close()

# streamlit_sign_language.py

import streamlit as st
import serial
import joblib
import numpy as np
import time

# Page title
st.set_page_config(page_title="Sign Language Recognizer", layout="centered")
st.title("🖐️ Real-Time Sign Language Recognizer")
st.markdown("Predicting sign language gestures using 5 MPU6050 sensors.")

# Sidebar settings
port = st.sidebar.text_input("Enter Serial Port", "COM10")
baud = st.sidebar.number_input("Baud Rate", value=115200)
start = st.sidebar.button("Start Prediction")

# Load model
model = joblib.load("Signed_Lang_Model_New1.pkl")

# Prediction display area
pred_area = st.empty()
raw_data_area = st.empty()

# Start reading from serial
if start:
    try:
        ser = serial.Serial(port, baud)
        time.sleep(2)
        st.sidebar.success(f"Connected to {port} 🎉")

        while True:
            if ser.in_waiting:
                line = ser.readline().decode("utf-8").strip()
                raw_data_area.markdown(f"**Raw Input:** `{line}`")

                parts = line.split()
                try:
                    data = np.array([float(x) for x in parts]).reshape(1, -1)

                    # Predict using model
                    prediction = model.predict(data)[0]
                    pred_area.markdown(f"### 🧠 Predicted Gesture: **{prediction}**")

                except ValueError:
                    pred_area.warning("Invalid data received. Waiting for valid input...")

    except Exception as e:
        st.error(f"🚫 Error: {e}")
