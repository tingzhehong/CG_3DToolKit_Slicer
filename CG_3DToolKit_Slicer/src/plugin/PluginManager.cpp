#include "PluginManager.h"
#include <QMessageBox>
#include <QPluginLoader>

PluginManager::PluginManager(QObject *parent) : QObject(parent)
{

}

bool PluginManager::LoadPlugin(QString path)
{
    QDir PluginDir(path);

    if (!PluginDir.exists())
    {
        QMessageBox::warning(NULL, QObject::tr(u8"信息"), QObject::tr(u8"找不到算子插件！"));
        return false;
    }

    foreach(QFileInfo infor, PluginDir.entryInfoList(QDir::Files))
    {
        if (infor.suffix().toLower() == "dll")
        {
            QString filepath = infor.filePath();
            QPluginLoader loader(filepath);
            QObject *instance = loader.instance();

            if (instance)
            {
                AlgorithmInterface *plugin = qobject_cast<AlgorithmInterface*>(instance);
                CG_NODEBLOCK *object = plugin->CreatAlgorithmPlugin();
                QString name = plugin->AlgorithmPluginName();
                QString ver = plugin->AlogorithmPlugVersion();
                int id = plugin->AlgorithmPluginID();

                m_Plugins.append(plugin);
                m_PluginObjects.append(object);

                if (plugin->m_Type == CG_ALGORITHM_TYPE::ALG2D)
                    m_PluginNames2D.append(name);
                if (plugin->m_Type == CG_ALGORITHM_TYPE::ALG3D)
                    m_PluginNames3D.append(name);

                m_PluginIDs.append(id);
                m_PluginVersions.append(ver);
            }
            else
            {
                QMessageBox::warning(NULL, QObject::tr(u8"信息"), QObject::tr(u8"加载算法插件失败！"));
                return false;
            }
        }
    }
    return true;
}

QList<AlgorithmInterface *> PluginManager::PluginList() const
{
    return m_Plugins;
}
