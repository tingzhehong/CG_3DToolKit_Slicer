#ifndef PLUGINMANAGER_H
#define PLUGINMANAGER_H

#include <QObject>
#include <QList>
#include <QDir>
#include <AlgorithmInterface.h>

class PluginManager : public QObject
{
    Q_OBJECT

public:
    explicit PluginManager(QObject *parent = nullptr);

public:
    bool LoadPlugin(QString path);

    QList<AlgorithmInterface *> PluginList() const;
    QList<AlgorithmInterface *> m_Plugins;
    QList<CG_NODEBLOCK *> m_PluginObjects;
    QList<int> m_PluginIDs;
    QStringList m_PluginNames2D;
    QStringList m_PluginNames3D;
    QStringList m_PluginVersions;
};

#endif // PLUGINMANAGER_H
