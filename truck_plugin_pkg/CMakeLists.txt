cmake_minimum_required(VERSION 3.8)
project(truck_plugin_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED) #gz-cmake3 è una dipendenza che ti serve per usare il comando gz_find_package, 
                                  #che è un'estensione di find_package specifica per i pacchetti Gazebo.

#questo serve per poter includere la libreria System, che è la libreria da cui ereditiamo per creare il nostro plugin.
gz_find_package(gz-sim8 REQUIRED) #include "gz/sim/System.hh"
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

# Non tutti quegli header richiedono un gz_find_package(...) esplicito separato nel tuo CMakeLists.txt.
# Una cosa è trovare gli header, un’altra è linkare le librerie necessarie.
# Un header può essere trovato anche se non hai scritto un gz_find_package dedicato, se il path include è 
# già arrivato indirettamente da un’altra dipendenza. In questo caso da gz-sim8 importa altre dipendenze transitive
# che non è necessario dichiarare esplicitamente, ma è comunque buona pratica farlo per chiarezza e per evitare problemi 
# in futuro se le dipendenze transitive dovessero cambiare.

# OPTIONAL
# Find gz-math
gz_find_package(gz-math7 REQUIRED COMPONENTS eigen3)
set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})

#OPTIONAL
# Find gz-msgs
gz_find_package(gz-msgs10 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

#OPTIONAL
# Find gz-plugin
gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

#OPTIONAL
# Find gz-transport
gz_find_package(gz-transport13 REQUIRED COMPONENTS log parameters)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})


add_library(truck_plugin SHARED src/TruckPlugin.cc) #crea una libreria condivisa di nome libtruck_plugin.so 

#questo ci permette di includere i file header che si trovano nella cartella include del nostro pacchetto, 
#sia quando stiamo costruendo il pacchetto (BUILD_INTERFACE) sia quando lo stiamo usando dopo l'installazione (INSTALL_INTERFACE)
target_include_directories(truck_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> #cioè qua dice la cartella che include il CMakeLists.txt e poi vai in include
  $<INSTALL_INTERFACE:include>)

# qua colleghiamo il nostro plugin alle diverse librerie trovate da gz_find_package
target_link_libraries(truck_plugin 
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER} #OPTIONAL
  gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER} #OPTIONAL
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER} #OPTIONAL
  gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER} #OPTIONAL
)


#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  # the following line skips the linter which checks for copyrights
#  # comment the line when a copyright and license is added to all source files
#  set(ament_cmake_copyright_FOUND TRUE)
#  # the following line skips cpplint (only works in a git repo)
#  # comment the line when this package is in a git repo and when
#  # a copyright and license is added to all source files
#  set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
#endif()

install(TARGETS truck_plugin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY include/
        DESTINATION include
)

install(DIRECTORY
launch
config
urdf
DESTINATION share/${PROJECT_NAME}
)

ament_package()
