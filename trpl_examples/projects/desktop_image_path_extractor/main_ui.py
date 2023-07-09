import os
import sys

import cv2
from PySide6.QtGui import QPixmap, QImage, Qt, QBrush, QColor, QFont
from PySide6.QtWidgets import QMainWindow, QApplication, QFileDialog, QLabel, QPushButton, QVBoxLayout, QSlider, QWidget, QHBoxLayout, QGraphicsScene, QGraphicsView, QGraphicsPixmapItem

from path_to_svg import convert_paths_to_svg
from path_extractor import extract
from common import pathsList_to_cv_image, get_existing_pathsList_JSON, save_pathsList_to_JSON_file

# read the image
image = cv2.imread('assets/za6_v2.png')
#image = cv2.imread('E:/GitHub/svg_paths_py/with_ui/assets/sect.png')

# Get the size of the image
height, width, channels = image.shape
# print(width, height)

# Initially an image will be resized to fit within the dimensions set below
INIT_MIN_WIDTH = 150
INIT_MIN_HEIGHT = 150

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()

        self.previewExtractMode = False
        self.isExtracting = False
        
        self.bold_font = QFont()
        self.bold_font.setBold(True)
        
        # image width/height Offsets
        self.scale_w = 1
        self.scale_h = 1

        self.max_canny_threshold_1 = 100
        self.max_canny_threshold_2 = 100
        self.curr_canny_threshold_1 = 50
        self.curr_canny_threshold_2 = 50

        self.max_scale_w = 4000
        self.max_scale_h = 4000
        # self.max_scale_w = height
        # self.max_scale_h = width
        self.max_scale = 4000

        # extract width and height
        self.xheight = 0
        self.xwidth = 0

        self.title = "Image Viewer"
        self.setWindowTitle(self.title)
        self.resize(500, 500)
        
        # Create a QGraphicsView widget
        self.view = QGraphicsView()
        self.view_horizontalScrollBar = self.view.horizontalScrollBar()
        self.view_verticalScrollBar = self.view.verticalScrollBar()

        self.mainLayout = QVBoxLayout()

        # self.image_cv2_origin = image
        self.image_cv2_origin = image  # cv_strict_resize(image, INIT_MIN_WIDTH, INIT_MIN_HEIGHT)
        

        
        self.image_layout = QHBoxLayout()  # To be added to main layout
        # self.label_image = QLabel(self)
        self.image_scene = QGraphicsScene(0, 0, 800, 600)
        self.pixmap_image = 0 # Image to render
        
        
        # self.label_image.setScaledConscrtents(False)
        # self.image_layout.addWidget(self.label_image)
        self.image_layout.addWidget(self.view)
        self.view.setScene(self.image_scene)
        brush = QBrush(QColor("black"))
        self.view.setBackgroundBrush(brush)
        self.view.fitInView(self.image_scene.sceneRect())
        self.mainLayout.addLayout(self.image_layout)
        # self.mainLayout.addLayout(self.view)
        
        self.loading_layout = QHBoxLayout()
        self.loading_label = QLabel("(Wait) Extracting paths...")
        self.loading_label.setFont(self.bold_font)
        self.loading_label.setStyleSheet('color: red;')
        self.loading_label.hide()
        self.loading_layout.addWidget(self.loading_label)
        self.mainLayout.addLayout(self.loading_layout)

        #--------------------------------------------------------------

        self.canny_threshold_1_layout_slider = QHBoxLayout()
        self.canny_threshold_1_sl = QSlider(Qt.Horizontal)
        self.canny_threshold_1_sl_label = QLabel("Threshold1 ("+str(self.curr_canny_threshold_1)+"):")
        self.canny_threshold_1_layout_slider.addWidget(self.canny_threshold_1_sl_label)
        self.canny_threshold_1_slider_handler()

        self.canny_threshold_2_layout_slider = QHBoxLayout()
        self.canny_threshold_2_sl = QSlider(Qt.Horizontal)
        self.canny_threshold_2_sl_label = QLabel("Threshold2 ("+str(self.curr_canny_threshold_2)+"):")
        self.canny_threshold_2_layout_slider.addWidget(self.canny_threshold_2_sl_label)
        self.canny_threshold_2_slider_handler()

        #--------------------------------------------------------------
        
        self.image_dimension_layout = QHBoxLayout()
        
        self.image_dimension_width_label = QLabel("Width: ")
        self.image_dimension_layout.addWidget(self.image_dimension_width_label)
        self.mainLayout.addLayout(self.image_dimension_layout)

        self.image_dimension_height_label = QLabel("Height: ")
        self.image_dimension_layout.addWidget(self.image_dimension_height_label)
        self.mainLayout.addLayout(self.image_dimension_layout)
        
        #----------------------------------------------------------------

        # Render image
        self.image = self.scale_cv_image_WH(self.image_cv2_origin, INIT_MIN_WIDTH)
        self.update_image_view()

        # ---------------------------------------------------------------
        self.scale_layout_slider = QHBoxLayout()
        self.scale_sl = QSlider(Qt.Horizontal)
        self.scale_sl_label = QLabel("scale(W:H)")
        self.scale_layout_slider.addWidget(self.scale_sl_label)
        self.scale_slider_handler()

        # Height slider
        # self.scale_width_layout_slider = QHBoxLayout()
        # self.scale_width_sl = QSlider(Qt.Horizontal)
        # self.scale_width_sl_label = QLabel("Height")
        # self.scale_width_layout_slider.addWidget(self.scale_width_sl_label)
        # self.scale_width_slider_handler()

        # Width slider
        # self.scale_height_layout_slider = QHBoxLayout()
        # self.scale_height_sl = QSlider(Qt.Horizontal)
        # self.scale_height_sl_label = QLabel("Width")
        # self.scale_height_layout_slider.addWidget(self.scale_height_sl_label)
        # self.scale_height_slider_handler()

        #----------------------------------------------------------------------------
        self.buttons_layout = QHBoxLayout()

        # Extract paths button
        self.extract_button = QPushButton("Extract Paths")
        self.extract_button.setFixedWidth(100)
        self.extract_button.clicked.connect(self.onExtract_paths)
        self.buttons_layout.addWidget(self.extract_button)
        self.mainLayout.addLayout(self.buttons_layout)

        # Import Image button
        self.import_button = QPushButton("Import Image")
        self.import_button.setFixedWidth(100)
        self.import_button.clicked.connect(self.onImport_image)
        self.buttons_layout.addWidget(self.import_button)
        self.mainLayout.addLayout(self.buttons_layout)

        # Import JSON extract button
        self.import_extract_button = QPushButton("Import JSON Extract")
        self.import_extract_button.setFixedWidth(150)
        self.import_extract_button.clicked.connect(self.onImport_extract)
        self.buttons_layout.addWidget(self.import_extract_button)
        self.mainLayout.addLayout(self.buttons_layout)



        # Set the alignment property of the view to enable automatic adjustment of scroll bars
        self.view.setAlignment(Qt.AlignLeft | Qt.AlignTop)

        widget = QWidget()
        widget.setLayout(self.mainLayout)
        self.setCentralWidget(widget)

    def get_image_cv(self, fileName):
        _image = cv2.imread(fileName)
        if (_image is not None):
            return _image
        else:
            print("Image is None")
            return

    def onExtract_paths(self):
        self.extract_paths_handler()

    def onImport_extract(self):
        self.onPreview_extract_mode(True)
        self.import_json_paths_extract_handler()
    
    def onImport_image(self):
        self.onPreview_extract_mode(False)
        self.import_image_handler()
    
    def onPreview_extract_mode(self, mode):
        self.previewExtractMode = mode
        if mode:
            self.extract_button.hide()
        else:
            self.extract_button.show()

    def extract_paths_handler(self):
        self.isExtracting = True
        self.loading_label.show()

        # self.image, final_paths = self.extract(self.scale_cv_image(self.image_cv2_origin))
        self.image = cv2.resize(self.image_cv2_origin, (self.xwidth, self.xheight), interpolation=cv2.INTER_AREA)
        self.image, final_paths, non_mirror_final_paths = extract(self.image, self.curr_canny_threshold_1, self.curr_canny_threshold_2)
        self.update_image_view()
        
        # Saving path data
        save_pathsList_to_JSON_file(final_paths)  # Json path data (mirrored paths)
        save_pathsList_to_JSON_file(non_mirror_final_paths, "output/non_mirror_pathsMap_extract.json")  # JSON path data (not mirrored)
        convert_paths_to_svg(non_mirror_final_paths, "output/non_mirror_pathsMap_extract.svg")   # SVG path data (mirrored paths)
        convert_paths_to_svg(final_paths, "output/pathsMap_extract.svg")  # SVG path data (not mirrored)

        self.isExtracting = False
        self.loading_label.hide()


    def import_image_handler(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "Open Image", "", "Image Files (*.png *.jpg)")
        if file_path:
            # Convert the path to a string with the correct file separator
            file_path = os.path.join(os.getcwd(), file_path)
            
            print("[import_image_handler()] File Path:", file_path)
            _image = self.get_image_cv(file_path)
            if _image is None:
                print("[import_image_handler()] Image is None ")
                return
            
           # _image = cv_strict_resize(_image, INIT_MIN_WIDTH, INIT_MIN_HEIGHT)
            self.scale_sl.setValue(INIT_MIN_WIDTH)  # reset scale slider
            self.image_cv2_origin = _image
            self.image = self.scale_cv_image_WH(self.image_cv2_origin, INIT_MIN_WIDTH)
        else:
            print("Something is wrong with image path")
            return
 
        self.update_image_view()
    
    def import_json_paths_extract_handler(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "Open Image", "", "JSON Files (*.json)")
        if file_path:
            # Convert the path to a string with the correct file separator
            file_path = os.path.join(os.getcwd(), file_path)
            
            print("[import_json_paths_extract_handler()] File Path:", file_path)
            
            pathsList = get_existing_pathsList_JSON(file_path)
            image = pathsList_to_cv_image(pathsList)
            if image is None:
                print("[import_json_paths_extract_handler()] Image is None ")
                return
            self.image_cv2_origin = image 
            self.image = self.scale_cv_image_WH(self.image_cv2_origin, INIT_MIN_WIDTH)
        else:
            print("[import_json_paths_extract_handler()] Something is wrong with image path")
            return
        
        self.update_image_view()
    
    def set_image_width_height_labels(self, width_, height_):
        self.xheight = height_
        self.xwidth = width_
        self.image_dimension_width_label.setText("Width: "+str(width_)+"pts")
        self.image_dimension_height_label.setText("Height: "+str(height_)+"pts")
    
    def scale_slider_handler(self):
        self.scale_sl.setMinimum(1)
        self.scale_sl.setMaximum(self.max_scale)
        self.scale_sl.setValue(INIT_MIN_WIDTH)
        self.scale_sl.setTickPosition(QSlider.TicksBelow)
        self.scale_sl.setTickInterval(1)
        self.scale_layout_slider.addWidget(self.scale_sl)
        self.scale_sl.valueChanged.connect(self.scale_image_view)
        self.mainLayout.addLayout(self.scale_layout_slider)
    
    def canny_threshold_1_slider_handler(self):
        self.canny_threshold_1_sl.setMinimum(1)
        self.canny_threshold_1_sl.setMaximum(self.max_canny_threshold_1)
        self.canny_threshold_1_sl.setValue(self.curr_canny_threshold_1)
        self.canny_threshold_1_sl.setTickPosition(QSlider.TicksBelow)
        self.canny_threshold_1_sl.setTickInterval(1)
        self.canny_threshold_1_layout_slider.addWidget(self.canny_threshold_1_sl)
        self.canny_threshold_1_sl.valueChanged.connect(self.on_change_canny_threshold_1)
        self.mainLayout.addLayout(self.canny_threshold_1_layout_slider)
    
    def canny_threshold_2_slider_handler(self):
        self.canny_threshold_2_sl.setMinimum(1)
        self.canny_threshold_2_sl.setMaximum(self.max_canny_threshold_2)
        self.canny_threshold_2_sl.setValue(self.curr_canny_threshold_2)
        self.canny_threshold_2_sl.setTickPosition(QSlider.TicksBelow)
        self.canny_threshold_2_sl.setTickInterval(1)
        self.canny_threshold_2_layout_slider.addWidget(self.canny_threshold_2_sl)
        self.canny_threshold_2_sl.valueChanged.connect(self.on_change_canny_threshold_2)
        self.mainLayout.addLayout(self.canny_threshold_2_layout_slider)

    def scale_width_slider_handler(self):
        self.scale_width_sl.setMinimum(1)
        self.scale_width_sl.setMaximum(self.max_scale_w)
        self.scale_width_sl.setValue(1)
        self.scale_width_sl.setTickPosition(QSlider.TicksBelow)
        self.scale_width_sl.setTickInterval(1)
        self.scale_width_layout_slider.addWidget(self.scale_width_sl)
        self.scale_width_sl.valueChanged.connect(self.scale_image_view_width)
        self.mainLayout.addLayout(self.scale_width_layout_slider)

    def scale_height_slider_handler(self):
        self.scale_height_sl.setMinimum(1)
        self.scale_height_sl.setMaximum(self.max_scale_h)
        self.scale_height_sl.setValue(1)
        self.scale_height_sl.setTickPosition(QSlider.TicksBelow)
        self.scale_height_sl.setTickInterval(1)
        self.scale_height_layout_slider.addWidget(self.scale_height_sl)
        self.scale_height_sl.valueChanged.connect(self.scale_image_view_height)
        self.mainLayout.addLayout(self.scale_height_layout_slider)
    
    def on_change_canny_threshold_1(self):
        self.curr_canny_threshold_1 = self.canny_threshold_1_sl.value()
        self.canny_threshold_1_sl_label.setText("Threshold1 ("+str(self.curr_canny_threshold_1)+"):")

        self.image = cv2.resize(self.image_cv2_origin, (self.xwidth, self.xheight), interpolation=cv2.INTER_AREA)
        self.image, _, _ = extract(self.image, self.curr_canny_threshold_1, self.curr_canny_threshold_2)
        self.update_image_view()
    
    def on_change_canny_threshold_2(self):
        self.curr_canny_threshold_2 = self.canny_threshold_2_sl.value()
        self.canny_threshold_2_sl_label.setText("Threshold2 ("+str(self.curr_canny_threshold_2)+"):")
        
        self.image = cv2.resize(self.image_cv2_origin, (self.xwidth, self.xheight), interpolation=cv2.INTER_AREA)
        self.image, _, _ = extract(self.image, self.curr_canny_threshold_1, self.curr_canny_threshold_2)
        self.update_image_view()

    def scale_image_view(self):
        self.scale_w = self.scale_sl.value()
        self.scale_h = self.scale_sl.value()

        self.image = self.scale_cv_image_WH(self.image_cv2_origin, self.scale_w)
        self.update_image_view()
    
    def scale_image_view_width(self):
        self.scale_w = self.scale_width_sl.value()
        self.image = self.scale_cv_image(self.image_cv2_origin, self.scale_h,  self.scale_w)
        self.update_image_view()

    def scale_image_view_height(self):
        self.scale_h = self.scale_height_sl.value()
        self.image = self.scale_cv_image(self.image_cv2_origin, self.scale_h, self.scale_w)

        self.update_image_view()
    
    def scale_cv_image(self, _image, scale_w=1, scale_h=1):
        if (scale_h <= 1 and scale_w <= 1):
            scale_h = self.scale_h
            scale_w = self.scale_w
         
        if _image is None:
            print("[scale_cv_image()] Image is None ")
            return
        height, width, _ = _image.shape
        
        self.set_image_width_height_labels(width+scale_w, height+scale_h)

        _image = cv2.resize(self.image_cv2_origin, (width+scale_w, height+scale_h), interpolation=cv2.INTER_AREA)
        return _image
    
    def scale_cv_image_WH(self, _image, scale_w=1):
        # if (scale_h <= 1 and scale_w <= 1):
        #     scale_h = self.scale_h
        #     scale_w = self.scale_w
         
        if _image is None:
            print("[scale_cv_image()] Image is None ")
            return
        height, width, _ = _image.shape
        _ratio = width/height
        if (width > height):
            _ratio = height/width
        
        if width > height:
            new_w = scale_w  # +width
            new_h = int(new_w*_ratio)
        else:
            new_h = scale_w  # +width
            new_w = int(new_h*_ratio)
        
        # print(width, height, "RS:", new_w, _ratio)
        self.set_image_width_height_labels(new_w, new_h)

        _image = cv2.resize(self.image_cv2_origin, (new_w, new_h), interpolation=cv2.INTER_AREA)
        return _image

    # @QtCore.pyqtSlot()
    def update_image_view(self):
        if self.pixmap_image != 0:
            self.image_scene.removeItem(self.pixmap_image)
        self.pixmap_image = self.cvImage_to_QI_or_QP(self.image)
        
        self.image_scene.addItem(self.pixmap_image)
    
    def cvImage_to_QI_or_QP(self, _image, rType="QP"):
        if _image is None:
            print("[cvImage_to_QI_or_QP()] Image is None ")
            return
        bytesPerLine = 3 * _image.shape[1]
        # self.self.image.shape => [0:height, 1:width, 2:channels]
        _image = QImage(_image.data, _image.shape[1], _image.shape[0], bytesPerLine, QImage.Format_RGB888)
        if rType == "QI":
            return _image
        _image = QGraphicsPixmapItem(QPixmap.fromImage(_image))
        return _image

    # Override showEvent function to make sure the scroll bars are set to 0
    def showEvent(self, event):
        # Set the values of the scroll bars to zero to ensure that they start at the beginning
        self.view_horizontalScrollBar.setValue(0)
        self.view_verticalScrollBar.setValue(0)
        super().showEvent(event)
    
    def scroll_bar_changed(self, sender):
        if sender == "vbar":
            print("Vertical scroll bar value:", self.view_verticalScrollBar.value())
        elif sender == "hbar":
            print("Horizontal scroll bar value:", self.view_horizontalScrollBar.value())


app = QApplication(sys.argv)
win = MainWindow()
win.show()
sys.exit(app.exec())
