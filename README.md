# christmas-tree-disco-sw

Code Composer Studio V7 Project for the EK-TM4C-123GXL LaunchPad
http://www.ti.com/tool/ccstudio
https://www.digikey.com/products/en?keywords=EK-TM4C-123GXL

Requires TivaWare from:
http://www.ti.com/tool/sw-tm4c?DCMP=tivac-series&HQS=tivaware

Setup PATH variable to link to TivaWare by right-clicking on the project in project explorer. Then click Properties.

Click the Resource->Linked Resources menu add a new Path Variable "COM_TI_TM4C_INSTALL_DIR" with a value of "C:\ti\TivaWare_C_Series-2.1.4.178" or where ever you have installed it.

Then from the Build->ARM Compiler->Include Options menu add a new #include search path "${COM_TI_TM4C_INSTALL_DIR}"