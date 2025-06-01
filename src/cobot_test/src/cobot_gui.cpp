#include <QApplication>
#include <QPushButton>
#include <QLCDNumber>
#include <QLabel>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QWidget>
#include <QProcess>
#include <QTimer>
#include <QHBoxLayout>
#include <QPalette>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <signal.h>
#include <unistd.h>

class CobotGUI : public QWidget {
    Q_OBJECT

public:
    CobotGUI(ros::NodeHandle& nh, const QString& nodeType, QWidget *parent = nullptr)
        : QWidget(parent), nodeProcess(nullptr), isPaused(false) {
        QVBoxLayout *mainLayout = new QVBoxLayout(this);

        QLabel *title = new QLabel("UR3 Parcel Sorting");
        title->setAlignment(Qt::AlignCenter);
        title->setStyleSheet("font-size: 22px; font-weight: bold; color: #2c3e50;");
        mainLayout->addWidget(title);

        // Total Volume LCD
        QGroupBox *totalBox = new QGroupBox("Total Volume");
        QVBoxLayout *totalLayout = new QVBoxLayout();
        lcdTotal = new QLCDNumber(); lcdTotal->display(3);
        lcdTotal->setStyleSheet("background-color: black; color: red; border: 3px solid darkred;");
        lcdTotal->setDigitCount(2);
        lcdTotal->setFixedHeight(60);
        lcdTotal->setFixedWidth(120);
        QLabel *totalLabel = new QLabel("Total Parcels");
        totalLabel->setAlignment(Qt::AlignCenter);
        totalLayout->addWidget(lcdTotal);
        totalLayout->addWidget(totalLabel);
        totalBox->setLayout(totalLayout);
        mainLayout->addWidget(totalBox, 0, Qt::AlignHCenter);

        QGroupBox *lcdBox = new QGroupBox("Parcel Counters");
        QHBoxLayout *lcdLayout = new QHBoxLayout();

        lcdRegular = new QLCDNumber(); lcdRegular->display(2);
        lcdExpress = new QLCDNumber(); lcdExpress->display(1);
        lcdFragile = new QLCDNumber(); lcdFragile->display(1);

        QString lcdStyle = "background-color: black; color: lime; border: 2px solid gray;";
        lcdRegular->setStyleSheet(lcdStyle);
        lcdExpress->setStyleSheet(lcdStyle);
        lcdFragile->setStyleSheet(lcdStyle);

        lcdLayout->addWidget(makeLCDGroup("Regular", lcdRegular));
        lcdLayout->addWidget(makeLCDGroup("Express", lcdExpress));
        lcdLayout->addWidget(makeLCDGroup("Fragile", lcdFragile));

        lcdBox->setLayout(lcdLayout);
        mainLayout->addWidget(lcdBox);

        cameraLabel = new QLabel("Camera Detection: N/A");
        cameraLabel->setStyleSheet("font-size: 14px; color: #2980b9; font-weight: bold;");
        mainLayout->addWidget(cameraLabel);

        QHBoxLayout *buttonLayout = new QHBoxLayout();

        QPushButton *pauseBtn = new QPushButton("Pause/Resume");
        pauseBtn->setStyleSheet("background-color: #f39c12; color: white; font-size: 14px;");
        connect(pauseBtn, &QPushButton::clicked, this, &CobotGUI::togglePause);
        buttonLayout->addWidget(pauseBtn);

        QPushButton *stopBtn = new QPushButton("E-Stop");
        stopBtn->setStyleSheet("background-color: #c0392b; color: white; font-size: 14px;");
        connect(stopBtn, &QPushButton::clicked, this, &CobotGUI::stopNode);
        buttonLayout->addWidget(stopBtn);

        mainLayout->addLayout(buttonLayout);

        setLayout(mainLayout);
        resize(520, 400);
        setWindowTitle("UR3 Cobot GUI");

        parcel_sub = nh.subscribe("/parcel_types_detected", 10, &CobotGUI::parcelCallback, this);

        QTimer *rosTimer = new QTimer(this);
        connect(rosTimer, &QTimer::timeout, this, []() { ros::spinOnce(); });
        rosTimer->start(10);

        QString cmd;
        if (nodeType == "assigned") cmd = "pick_and_place_cobot_node";
        else if (nodeType == "pile") cmd = "pile_stack_node";
        else if (nodeType == "pyramid") cmd = "pyramid_stack_node";
        else cmd = "pick_and_place_cobot_node";

        nodeProcess = new QProcess(this);
        nodeProcess->start("bash", QStringList() << "-c" << "rosrun cobot_test " + cmd);
    }

private:
    QGroupBox* makeLCDGroup(const QString& label, QLCDNumber* lcd) {
        QVBoxLayout *vbox = new QVBoxLayout();
        QLabel *lbl = new QLabel(label);
        lbl->setAlignment(Qt::AlignCenter);
        vbox->addWidget(lcd);
        vbox->addWidget(lbl);
        QGroupBox *box = new QGroupBox();
        box->setLayout(vbox);
        return box;
    }

    void togglePause() {
        if (nodeProcess) {
            if (!isPaused) {
                ::kill(nodeProcess->processId(), SIGSTOP);
                isPaused = true;
            } else {
                ::kill(nodeProcess->processId(), SIGCONT);
                isPaused = false;
            }
        }
    }

    void stopNode() {
        if (nodeProcess) {
            ::kill(nodeProcess->processId(), SIGTERM);
            nodeProcess->waitForFinished();
            delete nodeProcess;
            nodeProcess = nullptr;
            isPaused = false;
        }
    }

    void parcelCallback(const std_msgs::String::ConstPtr& msg) {
        cameraLabel->setText("Camera Detection: " + QString::fromStdString(msg->data));
    }

    QProcess *nodeProcess;
    bool isPaused;
    ros::Subscriber parcel_sub;
    QLabel *cameraLabel;
    QLCDNumber *lcdRegular;
    QLCDNumber *lcdExpress;
    QLCDNumber *lcdFragile;
    QLCDNumber *lcdTotal;
};

#include "cobot_gui.moc"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "cobot_gui_node");
    ros::NodeHandle nh;

    QApplication app(argc, argv);
    QApplication::setStyle("Fusion");

    QString mode = (argc > 1) ? argv[1] : "assigned";
    CobotGUI gui(nh, mode);
    gui.show();

    return app.exec();
}
