# Microsoft Developer Studio Project File - Name="NeuralNet" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Console Application" 0x0103

CFG=NeuralNet - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "NeuralNet.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "NeuralNet.mak" CFG="NeuralNet - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "NeuralNet - Win32 Release" (based on "Win32 (x86) Console Application")
!MESSAGE "NeuralNet - Win32 Debug" (based on "Win32 (x86) Console Application")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
RSC=rc.exe

!IF  "$(CFG)" == "NeuralNet - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "Release"
# PROP Intermediate_Dir "Release"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_CONSOLE" /D "_MBCS" /YX /FD /c
# ADD CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_CONSOLE" /D "_MBCS" /Fp"../../Release/NeuralNet.pch" /YX /Fo"../../Release/" /Fd"../../Release/" /FD /c
# ADD BASE RSC /l 0x409 /d "NDEBUG"
# ADD RSC /l 0x409 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /machine:I386
# ADD LINK32 ../../Release/Neuron.lib ../../Release/Math.lib ../../Release/Clustering.lib /nologo /subsystem:console /dll /pdb:"../../Release/NeuralNet.pdb" /machine:I386 /out:"../../Release/NeuralNet.dll" /implib:"../../Release/NeuralNet.lib"
# SUBTRACT LINK32 /pdb:none

!ELSEIF  "$(CFG)" == "NeuralNet - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "Debug"
# PROP Intermediate_Dir "Debug"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_CONSOLE" /D "_MBCS" /YX /FD /GZ /c
# ADD CPP /nologo /MTd /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_CONSOLE" /D "_MBCS" /FR"../../Debug/" /Fp"../../Debug/NeuralNet.pch" /YX /Fo"../../Debug/" /Fd"../../Debug/" /FD /GZ /c
# ADD BASE RSC /l 0x409 /d "_DEBUG"
# ADD RSC /l 0x409 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /debug /machine:I386 /pdbtype:sept
# ADD LINK32 ../../Debug/Neuron.lib ../../Debug/Math.lib ../../Debug/Clustering.lib /nologo /subsystem:console /dll /pdb:"../../Debug/NeuralNet.pdb" /debug /machine:I386 /out:"../../Debug/NeuralNet.dll" /implib:"../../Debug/NeuralNet.lib" /pdbtype:sept
# SUBTRACT LINK32 /pdb:none

!ENDIF 

# Begin Target

# Name "NeuralNet - Win32 Release"
# Name "NeuralNet - Win32 Debug"
# Begin Group "Source Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=.\AnalyticTangent.cpp
# End Source File
# Begin Source File

SOURCE=.\BioInspired.cpp
# End Source File
# Begin Source File

SOURCE=.\Blackbox.cpp
# End Source File
# Begin Source File

SOURCE=.\FeedForward.cpp
# End Source File
# Begin Source File

SOURCE=.\Naysayer.cpp
# End Source File
# Begin Source File

SOURCE=.\NumericTangent.cpp
# End Source File
# Begin Source File

SOURCE=.\RadialBasisFunctions.cpp
# End Source File
# Begin Source File

SOURCE=.\RBFClassification.cpp
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE="..\..\..\..\Program Files\Microsoft Visual Studio\VC98\Include\BASETSD.H"
# End Source File
# Begin Source File

SOURCE=.\BioInspired.h
# End Source File
# Begin Source File

SOURCE=.\Models.h
# End Source File
# Begin Source File

SOURCE=.\NeuralNet.h
# End Source File
# Begin Source File

SOURCE=..\Neuron\Numerical\Numerical.h
# End Source File
# Begin Source File

SOURCE=..\..\portable.h
# End Source File
# Begin Source File

SOURCE=.\RadialBasisFunctions.h
# End Source File
# Begin Source File

SOURCE=.\RBFClassification.h
# End Source File
# Begin Source File

SOURCE=..\..\Libraries\Reporter\reporter.h
# End Source File
# Begin Source File

SOURCE=..\..\types.h
# End Source File
# Begin Source File

SOURCE=..\..\Libraries\Math\VectorMath.h
# End Source File
# End Group
# Begin Source File

SOURCE="C:\Program Files\Microsoft Visual Studio\VC98\Include\BASETSD.H"
# End Source File
# Begin Source File

SOURCE=..\..\Clustering\Cluster\Cluster.h
# End Source File
# Begin Source File

SOURCE=..\..\Clustering\kMeans\kMeansCluster.h
# End Source File
# Begin Source File

SOURCE=.\NetCommon.h
# End Source File
# Begin Source File

SOURCE=..\..\Libraries\Math\NumericalMath.h
# End Source File
# Begin Source File

SOURCE=..\..\Clustering\Patterns\Pattern.h
# End Source File
# End Target
# End Project
