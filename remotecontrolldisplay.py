from tkinter import *
from PIL import Image, ImageDraw, ImageTk
import customtkinter as ctk
import sys, time, math
import numpy as np
import serial, serial.tools.list_ports
import pygame as pygame
import yt_dlp
import os
"""
#CONFIGURE SERIAL PORT
ser = serial.Serial(
port='COM5',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)
ser.isOpen()"""

"""COLOUR SCHEME"""
# Navy: #2F4156
# Teal: #567C8D
# Sky Blue: #C8D9E6
# Beige: #F5EFEB
# White: #FFFFF

class MainApp:
    def __init__(self, window):
        # Setting up the window
        self.window = window
        self.window.title('SBFFs: Coin-Picking Robot')
        self.window.geometry('1300x700')
        self.window.configure(bg='gray13')
        self.window.minsize(600, 700)
        self.window.maxsize(600, 700)

        self.setup_base()
        self.setup_topframe()
        self.setup_bottomframe()

        # Initialize pygame mixer
        pygame.mixer.init()

    def setup_base(self):
        self.main_frame = ctk.CTkFrame(self.window, width = 600, height=700, fg_color="red", corner_radius=0)
        self.main_frame.pack(fill=BOTH, expand=True)
        self.main_frame.pack_propagate(False)

        # Add Play Button
        self.play_button = ctk.CTkButton(self.main_frame, text="Download & Play Music", command=self.download_and_play_audio)
        self.play_button.pack(pady=20)

    def setup_topframe(self):
        self.title = ctk.CTkFrame(self.main_frame, width = 600, height = 150, fg_color="blue", corner_radius=0)
        self.title.pack(side = TOP, fill = X)
        self.title.pack_propagate(False)

    def setup_bottomframe(self):
        self.bottom_frame = ctk.CTkFrame(self.main_frame, width = 600, height = 550, fg_color="yellow", corner_radius=0)
        self.bottom_frame.pack(side=BOTTOM, fill=X)
        self.bottom_frame.pack_propagate(False)

        self.speedometer = ctk.CTkFrame(self.bottom_frame, width = 600, height = 350, fg_color = "pink", corner_radius=0)
        self.speedometer.pack(side=TOP, fill=X)
        self.speedometer.pack_propagate(False)

        self.direction = ctk.CTkFrame(self.bottom_frame, width=600, height = 200, fg_color = "white", corner_radius=0)
        self.direction.pack(side=BOTTOM, fill=X)
        self.direction.pack_propagate(False)

    
    def download_and_play_audio(self):
        url = "https://www.youtube.com/watch?v=uxpDa-c-4Mc"
        audio_path = "audio.mp3"

        ydl_opts = {
            'format': 'bestaudio/best',
            'outtmpl': 'audio.mp3',
            'ffmpeg-location': r'C:\Users\xinyi\Downloads\ffmpeg-2025-03-31-git-35c091f4b7-full_build.7z\ffmpeg-2025-03-31-git-35c091f4b7-full_build\bin',
            'postprocessors': [{
                'key': 'FFmpegExtractAudio',
                'preferredcodec': 'mp3',
                'preferredquality': '192',
            }],
        }
        
        with yt_dlp.YoutubeDL(ydl_opts) as ydl:
            ydl.download([url])

        if os.path.exists(audio_path):
            print(f"Download complete! Playing {audio_path}")

            # Play music using pygame
            pygame.mixer.music.load(audio_path)
            pygame.mixer.music.play()

if __name__ == "__main__": 
    window = ctk.CTk()
    app = MainApp(window)
    window.mainloop()