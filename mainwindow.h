#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QElapsedTimer>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void update();

    void on_actionShow_graph_triggered(bool checked);

    void on_actionMove_drones_triggered();

    void on_actionQuit_triggered();

    void on_actionCredits_triggered();

private:
    /**
     * @brief loadJson
     * @param title
     * @return
     */
    bool loadJson(const QString& title);
    void createVoronoiMap();
    void createServersLinks();
    void fillDistanceArray();

    Ui::MainWindow *ui;
    QVector<QVector<float>> distanceArray;

    // to animate drones
    QTimer *timer;
    QElapsedTimer elapsedTimer;
};
#endif // MAINWINDOW_H
