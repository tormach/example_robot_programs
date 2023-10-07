import QtQuick 2.0
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.2
import pathpilot.core 1.0
import pathpilot.controls 1.0
import pathpilot.panels.main 1.0
import pathpilot.handlers 1.0

//import QtQuick.Dialogs 1.0
//import pathpilot.Dialogs 1.0


RowLayout {
  id: root

  SourcePanel {
    visible: true
    folded: false
    Layout.fillHeight: true
  }

  ColumnLayout {
    Layout.fillWidth: false
    Layout.preferredWidth: 500


    RowLayout {
      x:10
      y:10
      visible: Config.user.custom.isFileViewVisible
      Rectangle {
        width: 70
        height: 30
        border {
          color: "#708090"
          width: 2
        }
        Label {
          x:10
          y:8
          text: "< Back"
        }

        MouseArea {
          id: mouseAreaBack
          anchors.fill: parent
          onClicked: {
            onClicked: Config.user.custom.backTraceReq = true
          }
        }

      }
    }
    // File View
    RowLayout {
      id: fileViewContainer
      visible: Config.user.custom.isFileViewVisible

      Rectangle {
        width: 500
        height: 400
        //color: "black"
        border {
          color: "#708090"
          width: 5
        }

        Rectangle {
          x: 10
          y: 10

          width: parent.width-20
          height: parent.height - 20
          border {
            color: "#700090"
            width: 2
          }


          ListView {
            id: fileView
            anchors.fill: parent

            model: ListModel {
              id: filesListModel
            }
            delegate: Rectangle {
              width: parent.width-10
              height: 30
              border {
                color: "#708090"
                width: 2
              }
              Text {
                x:5
                y:5
                text: model.name
              }

              MouseArea {
                id: mouseAreaList
                anchors.fill: parent
                onClicked: {
                  Config.user.custom.currentFileReq = true
                  Config.user.custom.currentFileMetadata = model.path
                }
              }
            }

            orientation: ListView.Vertical

            property int frame: 0
              Timer {
                interval: 16 // Adjust the interval to control the rendering frequency (approximately 60 frames per second)

                running: true
                repeat: true

                onTriggered: {
                  fileView.frame++
                  // Perform rendering or animation updates here based on the current frame value
                }
              }

              onFrameChanged: {
                // Perform rendering or animation updates here based on the current frame value
                if (Config.user.custom.fileViewRenderReq){ // check if we should update the file explorer
                fileView.model.clear()
                //var fileList = JSON.parse(Config.user.custom.testFileList)
                var fileList = JSON.parse(Config.user.custom.viewFiles)
                if (fileList.files.length)
                {
                  for (var i = 0; i < fileList.files.length; i++) {
                    //var obj = JSON.parse(fileList[i])
                    var name = fileList.files[i].name
                    var path = fileList.files[i].path
                    //var name = "Nice"
                    filesListModel.append({
                    name: name,
                    path: path
                  })
                }
              }else {
              filesListModel.append({
              name: "Folder is empty or no supported files found",
              path: ""
            })
          }

          Config.user.custom.fileViewRenderReq = false
        }

      }
      ScrollBar.vertical: ScrollBar {
        policy: ScrollBar.AlwaysOn
        size: fileView.height / fileView.contentHeight
        position: fileView.visibleArea.yPosition
        anchors.right: fileView.right
        width: 10
      }

    }
  }

}

}




// Image preview
RowLayout {
  id: imagePreviewContainer
  visible: Config.user.custom.isImagePreviewVisible

  Rectangle {
    width: 500
    height: 350
    color: "black"
    border {
      color: "#708090"
      width: 5
    }
    radius: 5

    Rectangle {
      x: 10
      y: 10

      width: parent.width-20
      height: parent.height - 20

      color: "transparent"
      Flickable {
        anchors.fill: parent
        contentWidth: childItem.width
        contentHeight: childItem.height
        clip:true

        Image {
          id: childItem
          source: "data:image/jpeg;base64, " + Config.user.custom.imageView

        }
        Text {
          x: 160
          y: 160
          color: "Red"
          visible: Config.user.custom.installingPackages || false
          text: "(Wait) Installing packages..."
          font.weight: Font.Bold
        }

      }
    }


  }
}

Text {
  x: 160
  y: 160
  color: "orange"
  visible: Config.user.custom.readingSvgFile || false
  text: "(Wait) Reading paths in SVG... ("+ Config.user.custom.svgReaderStatus+")"
  font.weight: Font.Bold
}

RowLayout {
  Text {
    x: 160
    y: 160
    color: "white"
    text: `Smooth Paths (${Config.user.custom.addSmoothing?"ON": "OFF"}) :`
    font.weight: Font.Bold
    font.pointSize: 12
  }
  Switch {
    id: "smoothSwitch"
    text: ""
    checked: Config.user.custom.addSmoothing

    onCheckedChanged: () => {
      if (smoothSwitch.checked){
        Config.user.custom.addSmoothing = true
      }else{
        Config.user.custom.addSmoothing = false
      }
    }
  }
}

RowLayout {
  visible: !Config.user.custom.isFileViewVisible
  PathPilotLabel {
    text: "Scale W:H"
  }

  Slider {
    id: slider1
    from:10
    value:Config.user.custom.minWidth
    to:Config.user.custom.maxWidth
    snapMode: Slider.SnapAlways

    function changed()
    {
      Config.user.custom.widthScale = value
      Config.user.custom.widthReq = true
    }

    onValueChanged: changed()

  }
  PathPilotLabel {
    text: parseInt(Config.user.custom.widthScale) || Config.user.custom.minWidth
  }

}
RowLayout {
  visible: !Config.user.custom.isFileViewVisible
  PathPilotLabel {
    text: "Threshold 1"
  }
  Slider {
    id: th1
    from:Config.user.custom.minThreshold
    value: Config.user.custom.initThreshold1
    to:Config.user.custom.maxThreshold

    function changed()
    {
      Config.user.custom.threshold1Value = value
      Config.user.custom.threshold1Req = true
    }

    onValueChanged: changed()
  }
  PathPilotLabel {
    text: parseInt(Config.user.custom.threshold1Value) || Config.user.custom.initThreshold1
  }
}
RowLayout {
  visible: !Config.user.custom.isFileViewVisible
  PathPilotLabel {
    text: "Threshold 2"
  }
  Slider {
    id: th2
    from: Config.user.custom.minThreshold
    value: Config.user.custom.initThreshold2
    to:Config.user.custom.maxThreshold

    function changed()
    {
      Config.user.custom.threshold2Value = value
      Config.user.custom.threshold2Req = true
    }

    onValueChanged: changed()
  }
  PathPilotLabel {
    text: parseInt(Config.user.custom.threshold2Value) || Config.user.custom.initThreshold2
  }
}

RowLayout {
  visible: !Config.user.custom.isFileViewVisible
  PathPilotButton {
    //width: 500
    enabled: !Config.user.custom.isDrawing
    text: "Extract"
    onClicked: Config.user.custom.extractReq = true


    Led {
      x:122
      y:3
      width: 10
      height: 10
      value: Config.user.custom.extractingPaths

    }
  }



  PathPilotButton {
    enabled: !Config.user.custom.isDrawing
    text: "Import"
    onClicked: Config.user.custom.impImgReq = true
  }
  PathPilotButton {
    text: "Run"
    enabled: !Config.user.custom.isDrawing
    onClicked: Config.user.custom.drawReq = true
    Led {
      x:122
      y:3
      width: 10
      height: 10
      value: Config.user.custom.isDrawing

    }
  }
  PathPilotButton {
    text: "Stop"
    visible: false
    //onClicked: Config.user.custom.imp_json_req = true
    onClicked: Config.user.custom.eStop = True

    Led {
      x:122
      y:3
      width: 10
      height: 10
      value: false

    }
  }
}



RowLayout {

  PathPilotButton {
    width: 750

    visible: Config.user.custom.isFileViewVisible
    text: "Preview"

    onClicked: {
      Config.user.custom.isFileViewVisible = false
      Config.user.custom.isImagePreviewVisible = true
    }
  }
}




VerticalFiller {
}
}

PreviewPanel {
  visible: true
  Layout.fillWidth: true
  Layout.fillHeight: true
  Layout.preferredWidth: 500
}
}
