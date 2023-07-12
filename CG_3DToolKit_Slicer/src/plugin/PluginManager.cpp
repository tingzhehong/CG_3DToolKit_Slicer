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

                if (plugin->m_Type == CG_ALGORITHM_TYPE::ALG2D)
                {
                    m_Plugins2D.append(plugin);
                    m_PluginObjects2D.append(object);
                    m_PluginNames2D.append(name);
                    m_PluginIDs2D.append(id);
                    m_PluginVersions2D.append(ver);
                }
                if (plugin->m_Type == CG_ALGORITHM_TYPE::ALG3D)
                {
                    m_Plugins3D.append(plugin);
                    m_PluginObjects3D.append(object);
                    m_PluginNames3D.append(name);
                    m_PluginIDs3D.append(id);
                    m_PluginVersions2D.append(ver);
                }
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

QList<AlgorithmInterface *> PluginManager::PluginList2D() const
{
    return m_Plugins2D;
}

QList<AlgorithmInterface *> PluginManager::PluginList3D() const
{
    return m_Plugins3D;
}
