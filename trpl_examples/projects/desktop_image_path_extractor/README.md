# Image path extractor (Desktop version)
The main goal for this project is to have an easy way to extract paths from images. QT is being used for the UI and OpenCV for image processing.

### Below is an image showing how the UI looks like and what each component does.
![UI overview image](/projects/desktop_image_path_extractor/assets/softwareOveriew.png)


To get started you will have to set up a virtual environment (instruction bellow), install the necessary packages and then run the main_ui.py script.

# Virtual environment setup
For those new to virtual environments, a virtual environment (venv) helps isolate our project packages/dependencies from the rest of the system (The last thing we need is to install a package that will break other packages that are not related to this project). Below are the commands to setup a virtual environment.
### Linux commands
- (step 1 install python3-venv library): `sudo apt-get install python3-venv`

- (step 2 create virtual environment called ".venv"): `python3 -m venv .venv`

- (step 3 activate environment): `source .env/bin/activate`

### Windows commands
- (step 1 create virtual environment called ".venv"):  `python -m venv .venv` <strong>OR</strong> `py -3 -m venv .venv`
- (step 3 activate environment): `.env\Scripts\activate.bat`

## Deactivating virtual environment
Command: `deactivate`

## Installing packages in a virtual environment
### Linux
Command: `python3 -m pip install package-name`
### Windows
Command: `python -m pip install package-name` 
OR
Command: `py -m pip install package-name`

# Installing packages needed for this project
 - ```python3 -m pip install PySide6```
 - ```python3 -m pip install opencv-python```

# Future improvements (TODO):
- Use QML for the UI so that it's easy to maintain
- Have a dropdown of multiple edge detection algorithms (Currently using Canny algorithm implemented by OpenCV)
- Add a path cleaning tool (It will be nice to be able to clean out unnecessary paths or points)
