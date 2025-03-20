@echo off
::This file was created automatically by CrossIDE to compile with C51.
C:
cd "\Users\syafi\OneDrive\Documents\asmstuff\CrossIDE\Pro32\PIC32_JDY40_test_Master\"
"C:\Users\syafi\OneDrive\Documents\asmstuff\CrossIDE\Call51\Bin\c51.exe" --use-stdout  "C:\Users\syafi\OneDrive\Documents\asmstuff\CrossIDE\Pro32\PIC32_JDY40_test_Master\JDY40_test.c"
if not exist hex2mif.exe goto done
if exist JDY40_test.ihx hex2mif JDY40_test.ihx
if exist JDY40_test.hex hex2mif JDY40_test.hex
:done
echo done
echo Crosside_Action Set_Hex_File C:\Users\syafi\OneDrive\Documents\asmstuff\CrossIDE\Pro32\PIC32_JDY40_test_Master\JDY40_test.hex
