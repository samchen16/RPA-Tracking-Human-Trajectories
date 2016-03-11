#ifndef TRAJECTORYVISUALIZER_H
#define TRAJECTORYVISUALIZER_H

#include <QMainWindow>
#include <vector>
#include "loader.h"
#include "trajectory.h"
#include "graph.h"
#include "pointinfo.h"

namespace Ui {
class TrajectoryVisualizer;
}

class TrajectoryVisualizer : public QMainWindow
{
    Q_OBJECT

private:
    Ui::TrajectoryVisualizer *ui;
    Graph* graph;
    PointInfo * pointInfo;
    std::vector<Trajectory*>* trajectories;
    bool timeRangeFixed;

    void updateView();

public:
    explicit TrajectoryVisualizer(QWidget *parent = 0);
    ~TrajectoryVisualizer();

public slots:
    void loadFile();
private slots:
    void on_actionNew_triggered();
    void on_actionOpen_triggered();
    void on_actionSave_triggered();
    void on_actionDelete_Point_triggered();
    void on_actionAdd_Point_triggered();
    void on_endTimeSlider_sliderMoved(int position);
    void on_startTimeSlider_sliderMoved(int position);
    void on_checkBox_toggled(bool checked);

    void on_startTimeEdit_editingFinished();
    void on_endTimeEdit_editingFinished();

    void on_actionScore_triggered(bool checked);
};

#endif // TRAJECTORYVISUALIZER_H
