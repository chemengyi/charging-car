#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <QGroupBox>
#include <QProcess>
#include <QMessageBox>

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onChargeComplete();
    void onReturnStation();
    void onStartCharging();

    void onStopTarget();
    void onStopReturn();

private:
    void setupUI();
    void runScript(const QString &script, const QStringList &args = {});
    void stopDockerTopic(const QString &topicName);

    /* ✅ 样式函数声明 */
    QString groupStyle(const QString &color);
    QString btnStyle(const QString &color);

    /* UI */
    QGroupBox *groupCharge;
    QPushButton *btnCharge;

    QGroupBox *groupReturn;
    QLineEdit *editReturnX;
    QLineEdit *editReturnY;
    QPushButton *btnReturn;
    QPushButton *btnStopReturn;

    QGroupBox *groupTarget;
    QLineEdit *editTargetX;
    QLineEdit *editTargetY;
    QPushButton *btnTarget;
    QPushButton *btnStopTarget;

    QProcess *process;
    const QString scriptDir =
        "/home/yy/controller/shared_dir/Comprehensive_Road_Testing/bin/";
};

#endif // MAINWINDOW_H
