#!/usr/bin/env python3

# %%
# Importing Libraries
import os
from PIL import Image
import requests
import numpy as np
import streamlit as st
import plotly.graph_objects as go

# %%
# Fetch Data
resp = requests.get('http://10.13.60.114:8080/fetch', headers={"Content-Type": "application/json; charset=utf-8"})
datJson = resp.json()

# Execution
logoData = Image.open(os.environ.get('logoPath', '/home/logo.png'))
st.set_page_config(page_title="Direction Finding Tank", page_icon=logoData, layout="centered", initial_sidebar_state="expanded")
col1, col2 = st.columns(2)
with col1:
    st.image(logoData)
with col2:
    st.header('Direction Finding Tank')
st.markdown("""---""")
col1, col2 = st.columns(2)
st.sidebar.image(logoData)

st.sidebar.markdown("---")
st.sidebar.write('Operator\'s Area')
col1, col2 = st.sidebar.columns(2)
with col1:
    forwardButton = st.button('Forward')
    backButton = st.button('Backward')
    armButton = st.button('ARM')
with col2:
    rightButton = st.button('Right')
    leftButton = st.button('Left')
    stopButton = st.button('STOP')

dropper = st.selectbox('Sector Selection', ('One', 'Two'))


ampls = datJson['_amplitudes']
incr = (datJson['f2'] - datJson['f1'])/len(ampls)
ranger = (np.array(list(range(len(ampls)))) * incr) + datJson['f1']

# print(resp.json())
# print(resp.status_code)

trace = go.Scatter(x=ranger, y=ampls, mode='lines', name='Frequency Spectrum')
layout = go.Layout(title='Frequency Spectrum', xaxis=dict(title='Frequencies'), yaxis=dict(title='Amplitudes'))
fig = go.Figure(data=[trace], layout=layout)
st.plotly_chart(fig, use_container_width=True)



hide_streamlit_style = """
            <style>
            footer {visibility: hidden;}
            </style>
            """
st.markdown(hide_streamlit_style, unsafe_allow_html=True)
