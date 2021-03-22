//##########################################################################
//#                                                                        #
//#                      CLOUDCOMPARE PLUGIN: qM3C2                        #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                         COPYRIGHT: CNRS / OSUR                         #
//#                                                                        #
//##########################################################################

#ifndef M3C2_PLUGIN_COMMANDS_HEADER
#define M3C2_PLUGIN_COMMANDS_HEADER

//CloudCompare
#include "ccCommandLineInterface.h"
#include "ccHObjectCaster.h"

//Local
#include "qM3C2Process.h"

static const char COMMAND_M3C2[] = "M3C2";
static const char NX[] = "Nx";
static const char NY[] = "Ny";
static const char NZ[] = "Nz";

struct CommandM3C2 : public ccCommandLineInterface::Command
{
	CommandM3C2() : ccCommandLineInterface::Command("M3C2", COMMAND_M3C2) {}

	virtual bool process(ccCommandLineInterface& cmd) override
	{
        cmd.print("[M3C2]");
		if (cmd.arguments().empty())
		{
			return cmd.error(QString("Missing parameter: parameters filename after \"-%1\"").arg(COMMAND_M3C2));
		}

		//open specified file
		QString paramFilename(cmd.arguments().takeFirst());
		cmd.print(QString("Parameters file: '%1'").arg(paramFilename));

		if (cmd.clouds().size() < 2)
		{
			cmd.error("Not enough clouds loaded (2 or 3 are expected: cloud 1, cloud 2 and optionally some core points)");
			return false;
		}

		ccPointCloud* cloud1 = ccHObjectCaster::ToPointCloud(cmd.clouds()[0].pc);
		ccPointCloud* cloud2 = ccHObjectCaster::ToPointCloud(cmd.clouds()[1].pc);
		ccPointCloud* corePointsCloud = (cmd.clouds().size() > 2 ? cmd.clouds()[2].pc : nullptr);
        unsigned int Np;
        int sfIdx;
        CCCoreLib::ScalarField* Nx;
        CCCoreLib::ScalarField* Ny;
        CCCoreLib::ScalarField* Nz;
        bool ok = true;
        CCVector3 normal;

		//display dialog
		qM3C2Dialog dlg(cloud1, cloud2, nullptr);
        dlg.setCorePointsCloud(corePointsCloud);
		if (!dlg.loadParamsFromFile(paramFilename))
		{
			return false;
		}

		QString errorMessage;
		ccPointCloud* outputCloud = nullptr; //only necessary for the command line version in fact
        ccPointCloud* outputCloud2 = nullptr; //only necessary for the command line version in fact
        if (!qM3C2Process::Compute(dlg, errorMessage, outputCloud, outputCloud2, !cmd.silentMode(), cmd.widgetParent()))
		{
			return cmd.error(errorMessage);
		}

		if (outputCloud)
		{
            if (cmd.cloudExportFormat().contains("*.sbf"))
            {
                Np = outputCloud->size();
                cmd.print("Save outputCloud in Simple Binary File, convert normals to scalar fields");
                if (outputCloud->hasNormals())
                {
                    sfIdx = outputCloud->addScalarField(NX);
                    Nx = outputCloud->getScalarField(sfIdx);
                    if (!Nx->resizeSafe(Np, true, CCCoreLib::NAN_VALUE)) // try to add the Nx scalar field
                    {
                        cmd.print("Failed to allocate memory for outputCloud Nx!");
                        Nx->release();
                        Nx = nullptr;
                        ok = false;
                    }
                    if (ok) // Nx allocated, try to allocate Ny
                    {
                        sfIdx = outputCloud->addScalarField(NY);
                        Ny = outputCloud->getScalarField(sfIdx);
                        if (!Ny->resizeSafe(Np, true, CCCoreLib::NAN_VALUE)) // try to add the Ny scalar field
                        {
                            cmd.print("Failed to allocate memory for outputCloud Ny!");
                            Ny->release();
                            Ny = nullptr;
                            ok = false;
                        }
                    }
                    if (ok) // Nx and Ny allocated, try to allocate Nz
                    {
                        sfIdx = outputCloud->addScalarField(NZ);
                        Nz = outputCloud->getScalarField(sfIdx);
                        if (!Nx->resizeSafe(Np, true, CCCoreLib::NAN_VALUE)) // try to add the Nz scalar field
                        {
                            cmd.print("Failed to allocate memory for outputCloud Nz!");
                            Nz->release();
                            Nz = nullptr;
                            ok = false;
                        }
                    }
                    if (ok) // Nx, Ny and Nz allocated
                    {
                        cmd.print("Nx, Ny and Nz scalar fields allocated successfully (outputCoud)");
                        for (unsigned int index=0; index<Np; index++)
                        {
                            normal = outputCloud->getPointNormal(index);
                            Nx->setValue(index, normal[0]);
                            Ny->setValue(index, normal[1]);
                            Nz->setValue(index, normal[2]);
                        }
                    }
                    else
                    {
                        cmd.print("Nx, Ny and Nz scalar fields not added, normals not exported!");
                    }
                }
            }
			CLCloudDesc cloudDesc(outputCloud, cmd.clouds()[0].basename + QObject::tr("_M3C2"), cmd.clouds()[0].path);
			if (cmd.autoSaveMode())
			{
				QString errorStr = cmd.exportEntity(cloudDesc, QString(), 0, ccCommandLineInterface::ExportOption::ForceNoTimestamp);
				if (!errorStr.isEmpty())
				{
					cmd.error(errorStr);
				}
			}
			//add cloud to the current pool
			cmd.clouds().push_back(cloudDesc);
		}

        if (outputCloud2)
        {
            if (cmd.cloudExportFormat().contains("*.sbf"))
            {
                Np = outputCloud2->size();
                cmd.print("Save outputCloud2 in Simple Binary File, convert normals to scalar fields");
                if (outputCloud2->hasNormals())
                {
                    sfIdx = outputCloud2->addScalarField(NX);
                    Nx = outputCloud2->getScalarField(sfIdx);
                    if (!Nx->resizeSafe(Np, true, CCCoreLib::NAN_VALUE)) // try to add the Nx scalar field
                    {
                        cmd.print("Failed to allocate memory for outputCloud Nx!");
                        Nx->release();
                        Nx = nullptr;
                        ok = false;
                    }
                    if (ok) // Nx allocated, try to allocate Ny
                    {
                        sfIdx = outputCloud2->addScalarField(NY);
                        Ny = outputCloud2->getScalarField(sfIdx);
                        if (!Ny->resizeSafe(Np, true, CCCoreLib::NAN_VALUE)) // try to add the Ny scalar field
                        {
                            cmd.print("Failed to allocate memory for outputCloud Ny!");
                            Ny->release();
                            Ny = nullptr;
                            ok = false;
                        }
                    }
                    if (ok) // Nx and Ny allocated, try to allocate Nz
                    {
                        sfIdx = outputCloud2->addScalarField(NZ);
                        Nz = outputCloud2->getScalarField(sfIdx);
                        if (!Nz->resizeSafe(Np, true, CCCoreLib::NAN_VALUE)) // try to add the Nz scalar field
                        {
                            cmd.print("Failed to allocate memory for outputCloud Nz!");
                            Nz->release();
                            Nz = nullptr;
                            ok = false;
                        }
                    }
                    if (ok) // Nx, Ny and Nz allocated
                    {
                        cmd.print("Nx, Ny and Nz scalar fields allocated successfully (outputCloud2)");
                        for (unsigned int index=0; index<Np; index++)
                        {
                            normal = outputCloud2->getPointNormal(index);
                            Nx->setValue(index, normal[0]);
                            Ny->setValue(index, normal[1]);
                            Nz->setValue(index, normal[2]);
                        }
                    }
                    else
                    {
                        cmd.print("Nx, Ny and Nz scalar fields not added, normals not exported!");
                    }
                }
            }
            CLCloudDesc cloudDesc2(outputCloud2, cmd.clouds()[0].basename + QObject::tr("_M3C2_2"), cmd.clouds()[0].path);
            if (cmd.autoSaveMode())
            {
                QString errorStr = cmd.exportEntity(cloudDesc2, QString(), 0, ccCommandLineInterface::ExportOption::ForceNoTimestamp);
                if (!errorStr.isEmpty())
                {
                    cmd.error(errorStr);
                }
            }
            // add cloud to the current pool
            cmd.clouds().push_back(cloudDesc2);
        }

		return true;
	}
};

#endif //M3C2_PLUGIN_COMMANDS_HEADER
