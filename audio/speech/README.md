# Speech Example

This program demonstrates using Google's text-to-speech library (gTTS) and an MP3 player (mpg321) to convert a text string to an MP3 file and play it.  I've found it helpful to ask the robot to tell me what it's doing while programming certain tasks, e.g. training the Multigrip gripper.  

This example is discussed on the Tormach user forums here: https://forums.tormach.com/t/example-program-robot-speech/36

## Libraries Used
MP3 player mpg321: 	http://mpg321.sourceforge.net/
Google text-to-speech: 	https://gtts.readthedocs.io/en/latest/

## Known Limitations

 - gTTS doesn't work well with characters like ()!$, etc.  
- MP3 files are all saved in the same directory as the running program and will clutter up your program directory.

## Credit
Inspiration for this example comes from this post by **Akhil Goel**: https://www.geeksforgeeks.org/convert-text-speech-python/