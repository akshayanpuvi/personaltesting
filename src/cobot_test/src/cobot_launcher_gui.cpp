#include <QApplication>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <QLabel>
#include <QProcess>
#include <QMessageBox>

class LauncherGUI : public QWidget {
    Q_OBJECT

public:
    LauncherGUI(QWidget *parent) : QWidget(parent) {
        QVBoxLayout *layout = new QVBoxLayout(this);

        QLabel *title = new QLabel("UR3 Sorting Master");
        title->setAlignment(Qt::AlignCenter);
        title->setStyleSheet("font-size: 24px; font-weight: bold; color: #2c3e50;");
        layout->addWidget(title);

        QPushButton *assignedSortBtn = new QPushButton("Assigned Sort");
        assignedSortBtn->setStyleSheet("background-color: #27ae60; color: white; font-size: 16px; padding: 10px;");
        connect(assignedSortBtn, &QPushButton::clicked, this, [this]() { launchMode("assigned"); });
        layout->addWidget(assignedSortBtn);

        QPushButton *pileStackBtn = new QPushButton("Pile Stack");
        pileStackBtn->setStyleSheet("background-color: #f39c12; color: white; font-size: 16px; padding: 10px;");
        connect(pileStackBtn, &QPushButton::clicked, this, [this]() { launchMode("pile"); });
        layout->addWidget(pileStackBtn);

        QPushButton *pyramidStackBtn = new QPushButton("Pyramid Stack");
        pyramidStackBtn->setStyleSheet("background-color: #2980b9; color: white; font-size: 16px; padding: 10px;");
        connect(pyramidStackBtn, &QPushButton::clicked, this, [this]() { launchMode("pyramid"); });
        layout->addWidget(pyramidStackBtn);

        resize(350, 250);
        setWindowTitle("UR3 Sorting Master");
    }

private:
    void launchMode(const QString& mode) {
        QString command = "rosrun cobot_test cobot_gui " + mode;
        QProcess::startDetached("bash", QStringList() << "-c" << command);
        close();
    }
};

#include "cobot_launcher_gui.moc"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    QApplication::setStyle("Fusion");

    LauncherGUI window(nullptr);
    window.show();

    return app.exec();
}
