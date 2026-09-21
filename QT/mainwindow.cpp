#include "mainwindow.h"
#include <QVBoxLayout>
#include <QGridLayout>
#include <QFileInfo>
#include <QColor>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), process(nullptr)
{
    setupUI();
    setWindowTitle("充电控制面板");
    resize(520, 480);
}

MainWindow::~MainWindow()
{
    if (process)
        process->kill();
}

void MainWindow::setupUI()
{
    auto *central = new QWidget(this);
    setCentralWidget(central);

    auto *mainLayout = new QVBoxLayout(central);
    mainLayout->setSpacing(18);
    mainLayout->setContentsMargins(25, 25, 25, 25);

    /* ===== 标题 ===== */
    auto *title = new QLabel("🚗 自动驾驶充电控制");
    title->setAlignment(Qt::AlignCenter);
    title->setStyleSheet(
        "QLabel{font-size:22px;font-weight:bold;color:#2c3e50;padding:10px;}"
    );
    mainLayout->addWidget(title);

    /* ===== 充电完成 ===== */
    groupCharge = new QGroupBox("⚡ 充电完成");
    groupCharge->setStyleSheet(groupStyle("#27ae60"));
    auto *v1 = new QVBoxLayout(groupCharge);
    btnCharge = new QPushButton("✅ 充电完成");
    btnCharge->setFixedHeight(48);
    btnCharge->setStyleSheet(btnStyle("#27ae60"));
    v1->addWidget(btnCharge);
    mainLayout->addWidget(groupCharge);

    /* ===== 返回充电总站 ===== */
    groupReturn = new QGroupBox("🔙 返回充电总站");
    groupReturn->setStyleSheet(groupStyle("#e74c3c"));
    auto *g1 = new QGridLayout(groupReturn);

    editReturnX = new QLineEdit("5.966083");
    editReturnY = new QLineEdit("-39.846901");

    g1->addWidget(new QLabel("X:"), 0, 0);
    g1->addWidget(editReturnX, 0, 1);
    g1->addWidget(new QLabel("Y:"), 1, 0);
    g1->addWidget(editReturnY, 1, 1);

    btnReturn = new QPushButton("📍 发布返程位置");
    btnReturn->setFixedHeight(42);
    btnReturn->setStyleSheet(btnStyle("#e74c3c"));

    btnStopReturn = new QPushButton("🛑 停止发布");
    btnStopReturn->setFixedHeight(36);
    btnStopReturn->setStyleSheet(
        "QPushButton{background:#7f8c8d;color:white;border-radius:4px;}"
        "QPushButton:hover{background:#626567;}"
    );

    g1->addWidget(btnReturn, 2, 0, 1, 2);
    g1->addWidget(btnStopReturn, 3, 0, 1, 2);

    mainLayout->addWidget(groupReturn);

    /* ===== 开始充电 ===== */
    groupTarget = new QGroupBox("🔋 开始充电");
    groupTarget->setStyleSheet(groupStyle("#f39c12"));
    auto *g2 = new QGridLayout(groupTarget);

    editTargetX = new QLineEdit("5.72");
    editTargetY = new QLineEdit("-9.2");

    g2->addWidget(new QLabel("X:"), 0, 0);
    g2->addWidget(editTargetX, 0, 1);
    g2->addWidget(new QLabel("Y:"), 1, 0);
    g2->addWidget(editTargetY, 1, 1);

    btnTarget = new QPushButton("🎯 发布目标位置");
    btnTarget->setFixedHeight(42);
    btnTarget->setStyleSheet(btnStyle("#f39c12"));

    btnStopTarget = new QPushButton("🛑 停止发布");
    btnStopTarget->setFixedHeight(36);
    btnStopTarget->setStyleSheet(
        "QPushButton{background:#7f8c8d;color:white;border-radius:4px;}"
        "QPushButton:hover{background:#626567;}"
    );

    g2->addWidget(btnTarget, 2, 0, 1, 2);
    g2->addWidget(btnStopTarget, 3, 0, 1, 2);

    mainLayout->addWidget(groupTarget);

    /* ===== 信号 ===== */
    connect(btnCharge, &QPushButton::clicked, this, &MainWindow::onChargeComplete);
    connect(btnReturn, &QPushButton::clicked, this, &MainWindow::onReturnStation);
    connect(btnTarget, &QPushButton::clicked, this, &MainWindow::onStartCharging);

    connect(btnStopReturn, &QPushButton::clicked, this, &MainWindow::onStopReturn);
    connect(btnStopTarget, &QPushButton::clicked, this, &MainWindow::onStopTarget);
}

/* ===== 样式 ===== */
QString MainWindow::groupStyle(const QString &color)
{
    return QString(
        "QGroupBox{font-size:16px;font-weight:bold;border:2px solid %1;"
        "border-radius:8px;margin-top:10px;padding:10px;}"
        "QGroupBox::title{subcontrol-origin:margin;left:20px;}"
    ).arg(color);
}

QString MainWindow::btnStyle(const QString &color)
{
    QColor baseColor(color);
    QColor hoverColor = baseColor.lighter(110);

    return QString(
        "QPushButton{"
        "background:%1;color:white;border:none;border-radius:5px;"
        "font-size:15px;font-weight:bold;"
        "}"
        "QPushButton:hover{background:%2;}"
    ).arg(baseColor.name(), hoverColor.name());
}
/* ===== 启动脚本 ===== */
void MainWindow::runScript(const QString &script, const QStringList &args)
{
    QString path = scriptDir + script;
    if (!QFileInfo::exists(path)) {
        QMessageBox::critical(this, "错误", "脚本不存在:\n" + path);
        return;
    }

    if (process) {
        process->kill();
        delete process;
    }

    process = new QProcess(this);
    process->setWorkingDirectory(scriptDir);
    process->start("/bin/bash", QStringList() << path << args);
}

/* ===== 停止 Docker 内话题 ===== */
void MainWindow::stopDockerTopic(const QString &topicName)
{
    QString cmd = QString(
        "docker exec autoware-humble-4.0.2 pkill -f \"ros2 topic pub.*%1\""
    ).arg(topicName);

    QProcess p;
    p.start("bash", QStringList() << "-c" << cmd);
    p.waitForFinished();

    QMessageBox::information(this, "已停止",
        QString("已停止发布话题：%1").arg(topicName));
}

/* ===== 按钮逻辑 ===== */
void MainWindow::onChargeComplete()
{
    runScript("pub_charge_complete.sh");
    QMessageBox::information(this, "完成", "充电完成信号已发送");
}

void MainWindow::onReturnStation()
{
    runScript("pub_return_goal.sh",
              {editReturnX->text(), editReturnY->text()});
}

void MainWindow::onStartCharging()
{
    runScript("pub_target_vehicle.sh",
              {editTargetX->text(), editTargetY->text()});
}

void MainWindow::onStopReturn()
{
    stopDockerTopic("return_goal");
}

void MainWindow::onStopTarget()
{
    stopDockerTopic("target_vehicle");
}
