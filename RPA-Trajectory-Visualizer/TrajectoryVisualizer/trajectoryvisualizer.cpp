#include "trajectoryvisualizer.h"
#include "ui_trajectoryvisualizer.h"
#include <QTextStream>
#include <QMouseEvent>

TrajectoryVisualizer::TrajectoryVisualizer(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::TrajectoryVisualizer)
{
    ui->setupUi(this);
    trajectories = new std::vector<Trajectory*>();
    pointInfo = new PointInfo(ui);

    graph = ui->graph;
    graph->init(pointInfo, trajectories);
    //graph->setupScore(ui->posScoreEdit, ui->velScoreEdit);
    timeRangeFixed = false;
}

TrajectoryVisualizer::~TrajectoryVisualizer()
{
    delete ui;
    trajectories->clear();
    delete trajectories;
}

void TrajectoryVisualizer::loadFile()
{
    QTextStream(stdout) << "load file" << endl;
    Loader::loadTrajectories("test", trajectories);
    graph->setStartTime(0.0f);
    graph->setEndTime(graph->getMaxTime());
    updateView();

}

void TrajectoryVisualizer::on_actionNew_triggered()
{

QTextStream(stdout) << "new" << endl;
}

void TrajectoryVisualizer::on_actionOpen_triggered()
{
    QTextStream(stdout) << "open" << endl;
    loadFile();
}

void TrajectoryVisualizer::on_actionSave_triggered()
{
    QTextStream(stdout) << "save" << endl;
    Loader::saveTrajectories("../output.txt", trajectories);
}

void TrajectoryVisualizer::on_actionDelete_Point_triggered()
{
    std::pair<int, int>* selectedPt = graph->getSelectedPoint();
    QTextStream(stdout) << "deleted point " << QString::number(selectedPt->first) << " " << QString::number(selectedPt->second) << endl;
    std::vector<Point*>* positions = trajectories->at(selectedPt->first)->getPositions();
    std::vector<Point*>* velocities = trajectories->at(selectedPt->first)->getVelocities();
    std::vector<Point*>* colors = trajectories->at(selectedPt->first)->getColors();
    std::vector<float>* times = trajectories->at(selectedPt->first)->getTimes();

    positions->erase(positions->begin() + selectedPt->second);
    velocities->erase(velocities->begin() + selectedPt->second);
    colors->erase(colors->begin() + selectedPt->second);
    times->erase(times->begin() + selectedPt->second);
    graph->deselectPoint();
    updateView();
}


void TrajectoryVisualizer::on_actionAdd_Point_triggered()
{

}


void TrajectoryVisualizer::on_checkBox_toggled(bool checked)
{
    timeRangeFixed = checked;
}

void TrajectoryVisualizer::on_endTimeSlider_sliderMoved(int position)
{
    float graphPosition = position/99.0f * graph->getMaxTime();
    graph->setEndTime(graphPosition);
    updateView();
}

void TrajectoryVisualizer::on_startTimeSlider_sliderMoved(int position)
{
    float graphPosition = position/99.0f * graph->getMaxTime();
    graph->setStartTime(graphPosition);
    updateView();
}


void TrajectoryVisualizer::on_startTimeEdit_editingFinished()
{
    float graphTime = ui->startTimeEdit->text().toFloat();
    graph->setStartTime(graphTime);
    updateView();
}

void TrajectoryVisualizer::on_endTimeEdit_editingFinished()
{
    float time = ui->endTimeEdit->text().toFloat();
    graph->setEndTime(time);
    updateView();
}

void TrajectoryVisualizer::updateView()
{
    graph->updateView();
    float graphStartTime = graph->getStartTime();
    float graphEndTime = graph->getEndTime();
    float startTime = graphStartTime / graph->getMaxTime() * 99.0f;
    float endTime = graphEndTime / graph->getMaxTime() * 99.0f;

    ui->startTimeEdit->setText(QString::number(graphStartTime, 'f', 3));
    ui->endTimeEdit->setText(QString::number(graphEndTime, 'f', 3));
    ui->startTimeSlider->setValue(startTime);
    ui->endTimeSlider->setValue(endTime);
}

void TrajectoryVisualizer::on_actionScore_triggered(bool checked)
{
    if (graph->getSelected()) {
        QTextStream(stdout) << "view score set to " << checked << endl;
        graph->setViewScore(checked);
    }
    else {
        QTextStream(stdout) << "must first select point to view score" << endl;
        ui->actionScore->setChecked(false);
    }
}
