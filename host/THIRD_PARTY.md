# Third-party components

The portable Windows build dynamically links to Qt 6.8.3 Core, Gui, Widgets and SerialPort, and includes the Qt Windows platform plugin. Qt binaries were obtained from the official Qt SDK repository. Qt is available under commercial and open-source licenses; this distribution uses the LGPL v3 option for these modules. The application source and build instructions are included so it can be rebuilt or relinked. Do not replace Qt DLLs with an incompatible ABI.

- Qt source: https://download.qt.io/archive/qt/6.8/6.8.3/submodules/
- Qt licenses: https://doc.qt.io/qt-6/licensing.html
- LGPL v3: https://www.gnu.org/licenses/lgpl-3.0.html
- GPL v3 (incorporated by LGPL v3): https://www.gnu.org/licenses/gpl-3.0.html

The bundled MinGW GCC 13.1 runtime libraries are provided under their upstream GCC/MinGW runtime licenses and the GCC Runtime Library Exception where applicable.

- GCC Runtime Library Exception: https://www.gnu.org/licenses/gcc-exception-3.1.html
- MinGW-w64: https://www.mingw-w64.org/

Full license texts are included alongside the portable build in licenses/. Qt SDK/compiler are only needed to develop the application; the portable build contains the runtime files needed to launch it.

DualForge is copied from the user's existing desktop project and linked statically under its MIT license. Its unchanged source and license are included in third_party/dualforge.

SDL 3.4.18 is built from the official, SHA256-verified source archive and linked dynamically as SDL3.dll. SDL uses the zlib license. Upstream source: https://www.libsdl.org/release/SDL3-3.4.18.tar.gz . Its license and DualForge's license are included in portable/licenses/.
