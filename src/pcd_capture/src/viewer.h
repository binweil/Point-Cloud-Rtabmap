#ifndef VIEWER_H
#define VIEWER_H

#include <QMainWindow>

namespace Ui {
class Viewer;
}

class Viewer : public QMainWindow
{
    Q_OBJECT

public:
    explicit Viewer(QWidget *parent = nullptr);
    ~Viewer();

private:
    Ui::Viewer *ui;
};

#endif // VIEWER_H
