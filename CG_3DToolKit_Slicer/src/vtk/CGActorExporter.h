#ifndef ACTOREXPORTER_H
#define ACTOREXPORTER_H

#include <QRunnable>
#include <QString>

class vtkActor;
namespace CGVTKUtils
{

class ActorExporter : public QRunnable
{
public:
    ActorExporter(vtkActor* actor, const QString& file);

    void run();

protected:
    vtkActor* m_actor = nullptr;
    QString m_exportFile;
};

} // namespace CGVTKUtils
#endif // ACTOREXPORTER_H
