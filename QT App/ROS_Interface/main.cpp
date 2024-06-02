#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QDialog>
#include <QProcess>
#include <QDebug> // Include the header file for qDebug()
#include <QLabel> // Include the header file for QLabel

// Declare the executeRoomScript function before its use
void executeRoomScript(const QString &scriptPath) {
    QProcess::startDetached("gnome-terminal --working-directory=/home/ibrahim -- " + scriptPath);
}

class RoomSelectionWindow : public QDialog {
public:
    RoomSelectionWindow(QWidget *parent = nullptr) : QDialog(parent) {
        setWindowTitle("Goal Selection");
        setFixedSize(500, 400); // Resized the window to be bigger

        QVBoxLayout *layout = new QVBoxLayout(this);
        for (int i = 1; i <= 4; ++i) {
            QString roomLabel;
            QString scriptPath;

            // Assign different names and script paths based on the room number
            switch (i) {
                case 1:
                    roomLabel = "Control System Lab";
                    scriptPath = "/home/ibrahim/room1.sh";
                    break;
                case 2:
                    roomLabel = "EMS Lab";
                    scriptPath = "/home/ibrahim/room2.sh";
                    break;
                case 3:
                    roomLabel = "Digital/Embedded Lab";
                    scriptPath = "/home/ibrahim/room3.sh";
                    break;
                case 4:
                    roomLabel = "Signal Processing Lab";
                    scriptPath = "/home/ibrahim/room4.sh";
                    break;
            }

            QPushButton *roomButton = new QPushButton(roomLabel, this);
            layout->addWidget(roomButton);

            qDebug() << "Script path for Room" << i << ":" << scriptPath; // Debug output

            // Connect the clicked signal of the room button to a slot
            connect(roomButton, &QPushButton::clicked, this, [scriptPath]() {
                executeRoomScript(scriptPath);
            });
        }
    }
};

class CentralButtonApp : public QWidget {
public:
    CentralButtonApp(QWidget *parent = nullptr) : QWidget(parent) {
        setWindowTitle("LandMark Based Localisation App");
        resize(400, 400);

        // Create a label for the welcome message
        QLabel *welcomeLabel = new QLabel("Welcome to the app. Press the button to start", this);

        centralButton = new QPushButton(this);
        centralButton->setFixedSize(100, 100);
        centralButton->setStyleSheet("background-color: red");

        QHBoxLayout *layout = new QHBoxLayout(this);
        layout->addWidget(welcomeLabel); // Add the label to the layout
        layout->addStretch();
        layout->addWidget(centralButton);
        layout->addStretch();

        connect(centralButton, &QPushButton::clicked, this, &CentralButtonApp::openRoomSelectionWindow);
    }

private slots:
    void openRoomSelectionWindow() {
        // Open the room selection window as a pop-up dialog
        RoomSelectionWindow roomSelectionWindow;
        roomSelectionWindow.exec();
    }

private:
    QPushButton *centralButton;
};


int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    CentralButtonApp centralButtonApp;
    centralButtonApp.show();
    return app.exec();
}
