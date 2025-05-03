import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
from gtts import gTTS
import os
import pygame
import time

image_path = os.getcwd()

    
# Define a dictionary mapping text to image file paths
image_map = {
    "A": f"{image_path}/orbit/A_E_I.png",
    "B": f"{image_path}/orbit/B_M_P.png",
    "C": f"{image_path}/orbit/C_D_G_K_N_S_T_X_Y_Z.png",
    "D": f"{image_path}/orbit/C_D_G_K_N_S_T_X_Y_Z.png",
    "E": f"{image_path}/orbit/A_E_I.png",
    "F": f"{image_path}/orbit/F_V.png",
    "G": f"{image_path}/orbit/C_D_G_K_N_S_T_X_Y_Z.png",
    "H": f"{image_path}/orbit/TH.png",
    "I": f"{image_path}/orbit/A_E_I.png",
    "J": f"{image_path}/orbit/CH_J_SH.png",
    "K": f"{image_path}/orbit/C_D_G_K_N_S_T_X_Y_Z.png",
    "L": f"{image_path}/orbit/L.png",
    "M": f"{image_path}/orbit/B_M_P.png",
    "N": f"{image_path}/orbit/C_D_G_K_N_S_T_X_Y_Z.png",
    "O": f"{image_path}/orbit/O.png",
    "P": f"{image_path}/orbit/B_M_P.png",
    "Q": f"{image_path}/orbit/Q_W.png",
    "R": f"{image_path}/orbit/R.png",
    "S": f"{image_path}/orbit/C_D_G_K_N_S_T_X_Y_Z.png",
    "T": f"{image_path}/orbit/C_D_G_K_N_S_T_X_Y_Z.png",
    "U": f"{image_path}/orbit/O.png",
    "V": f"{image_path}/orbit/F_V.png",
    "W": f"{image_path}/orbit/Q_W.png",
    "X": f"{image_path}/orbit/C_D_G_K_N_S_T_X_Y_Z.png",
    "Y": f"{image_path}/orbit/C_D_G_K_N_S_T_X_Y_Z.png",
    "Z": f"{image_path}/orbit/C_D_G_K_N_S_T_X_Y_Z.png",
    "CH": f"{image_path}/orbit/CH_J_SH.png",
    "SH": f"{image_path}/orbit/CH_J_SH.png",
    "TH": f"{image_path}/orbit/TH.png",
    "a": f"{image_path}/orbit/A_E_I.png",
    "b": f"{image_path}/orbit/B_M_P.png",
    "c": f"{image_path}/orbit/C_D_G_K_N_S_T_X_Y_Z.png",
    "d": f"{image_path}/orbit/C_D_G_K_N_S_T_X_Y_Z.png",
    "e": f"{image_path}/orbit/A_E_I.png",
    "f": f"{image_path}/orbit/F_V.png",
    "g": f"{image_path}/orbit/C_D_G_K_N_S_T_X_Y_Z.png",
    "h": f"{image_path}/orbit/TH.png",
    "i": f"{image_path}/orbit/A_E_I.png",
    "j": f"{image_path}/orbit/CH_J_SH.png",
    "k": f"{image_path}/orbit/C_D_G_K_N_S_T_X_Y_Z.png",
    "l": f"{image_path}/orbit/L.png",
    "m": f"{image_path}/orbit/B_M_P.png",
    "n": f"{image_path}/orbit/C_D_G_K_N_S_T_X_Y_Z.png",
    "o": f"{image_path}/orbit/O.png",
    "p": f"{image_path}/orbit/B_M_P.png",
    "q": f"{image_path}/orbit/Q_W.png",
    "r": f"{image_path}/orbit/R.png",
    "s": f"{image_path}/orbit/C_D_G_K_N_S_T_X_Y_Z.png",
    "t": f"{image_path}/orbit/C_D_G_K_N_S_T_X_Y_Z.png",
    "u": f"{image_path}/orbit/O.png",
    "v": f"{image_path}/orbit/F_V.png",
    "w": f"{image_path}/orbit/Q_W.png",
    "x": f"{image_path}/orbit/C_D_G_K_N_S_T_X_Y_Z.png",
    "y": f"{image_path}/orbit/C_D_G_K_N_S_T_X_Y_Z.png",
    "z": f"{image_path}/orbit/C_D_G_K_N_S_T_X_Y_Z.png",
    "ch": f"{image_path}/orbit/CH_J_SH.png",
    "sh": f"{image_path}/orbit/CH_J_SH.png",
    "th": f"{image_path}/orbit/TH.png",
    ".": f"{image_path}/orbit/default.png",
    ",": f"{image_path}/orbit/default.png",
    "!": f"{image_path}/orbit/default.png",
    "?": f"{image_path}/orbit/default.png",
    "-": f"{image_path}/orbit/default.png",
    " ": f"{image_path}/orbit/default.png",

}


# Get the current script's directory
script_dir = os.path.dirname(os.path.abspath(__file__))  # Correct the reference to __file__
audio_folder = os.path.join(script_dir, "Audio")

# Variable to store the last entered text
last_entered_text = ""


def speech_update():
    global last_entered_text
    text = text_entry.get()

    if text != last_entered_text:
        last_entered_text = text

        # Create the "Audio" folder if it doesn't exist
        try:
            if not os.path.exists(audio_folder):
                os.makedirs(audio_folder)
        except OSError as e:
            print(f"Error creating the 'Audio' folder: {e}")

        # Create a unique audio filename based on a timestamp
        timestamp = int(time.time())
        audio_filename = os.path.join(audio_folder, f"output_{timestamp}.mp3")

        # Create a gTTS object and save the text as audio
        try:
            tts = gTTS(text)
            tts.save(audio_filename)
        except Exception as e:
            print(f"Error creating audio: {e}")


def show_lips():
    text = text_entry.get()
    lip_sync(text)
    play_audio()


def lip_sync(text):
    # Remove any existing images
    image_label.pack_forget()

    # Split the text into individual letters
    letters = [letter for letter in text if letter in image_map]

    if letters:
        display_images(letters, 0)


def display_images(letters, index):
    if index < len(letters):
        letter = letters[index]
        image_path = image_map.get(letter)
        if image_path:
            try:
                image = Image.open(image_path)
                image = image.resize((640, 360))
                photo = ImageTk.PhotoImage(image)

                image_label.config(image=photo)
                image_label.image = photo
                image_label.pack()

                # Schedule the next image after a delay (e.g., 500 milliseconds)
                app.after(25, display_images, letters, index + 1)
            except Exception as e:
                print(f"Error displaying image: {e}")
        else:
            print(f"Image not found for letter: {letter}")

    else:
        # Display the final image
        final_image_path = ""  # Replace with the path to your final image
        final_image = Image.open(final_image_path)
        final_photo = ImageTk.PhotoImage(final_image)

        image_label.config(image=final_photo)
        image_label.image = final_photo
        image_label.pack()


def play_audio():
    speech_update()
    play_speech()


def play_speech():
    # List audio files in ascending order
    audio_files = sorted([f for f in os.listdir(audio_folder) if f.endswith(".mp3")])

    if audio_files:
        latest_audio_file = os.path.join(audio_folder, audio_files[-1])
        print()
        try:
            pygame.mixer.init()
            pygame.mixer.music.load(latest_audio_file)
            pygame.mixer.music.play()
        except Exception as e:
            print(f"Error playing audio: {e}")


# Create the main application window
app = tk.Tk()
app.title("Modern Image Viewer")

# Set the initial size of the window
app.geometry("600x400")  # Width x Height

# Use themed widgets (ttk)
style = ttk.Style()
style.configure('TButton', foreground='blue', padding=10)
style.configure('TLabel', foreground='green')

# Create a text input field
text_label = ttk.Label(app, text="Enter text:")
text_label.pack()
text_entry = ttk.Entry(app)
text_entry.pack()

# Bind the update_audio function to the <<FocusOut>> event
text_entry.bind("<FocusOut>", lambda e: speech_update())

# Create a label to display the image
image_label = ttk.Label(app)

# Create a button with updated style
speak_button = ttk.Button(app, text="Speak", command=show_lips, style='TButton')
speak_button.pack()

app.mainloop()
