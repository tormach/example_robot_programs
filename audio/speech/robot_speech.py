import sh
import os
from robot_command.rpl import *

# import MP3 player
try:
    sh.dpkg_query("-W", "mpg321")
except:
    notify("The first time this program is run it must import two packages (MP3 player, Google's text-to-speech library).  "
    "This may take a minute or two depending on your internet connection.  Subsequent program runs will not experience this delay")
    with sh.contrib.sudo:
        sh.apt_get("install", "-y", "mpg321")

# import Google's text-to-speech package
try:
    from gtts import gTTS
except:
    from sh import pip3
    with sh.contrib.sudo:
        pip3.install("gtts")
    from gtts import gTTS

def speak(string):
    language = 'en'
    # create file name using the text that's being spoken
    filename = string + ".mp3"
    # no spaces in file name please
    filename = filename.replace(" ", "_")
    # don't convert text to MP3 if the MP3 already exists
    if os.path.exists(filename):
        # we've already converted this text, just play the MP3
        os.system("mpg321 " + filename)
    else:
        # need to create it
        myobj = gTTS(text=string, lang=language, slow=False)
        myobj.save(filename)
        os.system("mpg321 " + filename)

