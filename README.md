
---

# Repositorio Base: FRC 2026 - Swerve Drive REV Robotics

¡Bienvenido al repositorio base para la temporada de FRC 2026!

Este repositorio tiene como propósito ser **la plantilla inicial** para que los miembros del equipo construyan el código del robot basado en el sistema de **Swerve Drive** de **REV Robotics**. Aquí encontrarás la configuración y el código base que podrás clonar y utilizar como punto de partida para desarrollar y adaptar el código según las necesidades de nuestro robot para la temporada 2026.

## 🚗 ¿Qué es el Swerve Drive?

El **Swerve Drive** es un sistema avanzado de movimiento para robots que permite el control independiente de cada rueda, lo que otorga una mayor maniobrabilidad y precisión durante las competencias. Usamos el sistema de **REV Robotics** porque proporciona componentes de alta calidad y confiabilidad.

## 📂 Estructura del Repositorio

Este repositorio contiene:

* **Código Base del Swerve Drive**: La implementación inicial del sistema Swerve Drive utilizando los componentes de REV Robotics.
* **Configuración de Hardware**: Ajustes y configuraciones necesarias para trabajar con los motores y controladores de REV Robotics.
* **Plantilla de Proyecto**: Todo el código estructurado para que puedas clonarlo y construir tu propio código de robot encima.

## 🔧 ¿Cómo Empezar?

1. **Clona este repositorio** para crear una copia local en tu máquina:

   ```bash
   git clone https://github.com/Balam-Robotics/SwerveBalam.git
   ```

2. **Crea tu propia rama** para trabajar en tu parte del código. Esto asegura que los cambios no afecten el código base:

   ```bash
   git checkout -b año-nombre-del-creador
   git checkout -b 2026-gerardo
   ```

3. **Construye tu código**: Usa esta plantilla base para desarrollar el código de control del robot. Asegúrate de ajustar cualquier parámetro relacionado con los sensores, motores y otros componentes según tu configuración.

4. **Realiza cambios y prueba el robot**: A medida que desarrollas, realiza pruebas frecuentes para asegurarte de que todo esté funcionando correctamente.

5. **Envía tus cambios** a la rama principal del repositorio:

   ```bash
   git add .
   git commit -m "Descripción de los cambios realizados"
   git push origin nombre-de-tu-rama
   ```

6. **Solicita un pull request (PR)** para que tus cambios sean revisados y fusionados con el código base.

## ⚠️ Notas Importantes

* **Mantén el código limpio**: Es importante que el código sea bien estructurado, comentado y fácil de entender. Esto facilitará la colaboración dentro del equipo.
* **Pruebas frecuentes**: Realiza pruebas en cada fase del desarrollo para evitar problemas en etapas posteriores.
* **Configuración de Hardware**: Si tienes configuraciones especiales para el robot o los motores, asegúrate de actualizar los parámetros en los archivos correspondientes.

## 🚀 Estructura de Carpetas

* **src/**: El código fuente de la robot.
* **vendordeps/**: Archivos de configuración de hardware de terceros.

## 🛠️ ¿Qué Está Incluido?

* Configuración básica para controlar el sistema de Swerve Drive con motores REV.
* Plantilla de código para integrar otros sistemas (sensores, controladores, etc.).
* Funciones de movimiento básico, como desplazamiento en todas las direcciones y giros precisos.

## 🔄 Actualizaciones

Este repositorio será actualizado a medida que avance la temporada. Las actualizaciones incluirán mejoras en el código, ajustes para optimizar el rendimiento del robot y cualquier cambio relevante en la estructura del equipo.

## 📅 Cronograma

* **Inicio de la temporada**: Desarrollo del código base del Swerve Drive.
* **Fase de pruebas**: Validación del código y ajustes necesarios para optimizar el rendimiento del robot.
* **Competencia**: El código final será desplegado en la competencia de FRC 2026.

¡Gracias por ser parte de este proyecto! Con este repositorio, tendrás todo lo necesario para comenzar a construir el código del robot de la temporada FRC 2026. ¡Adelante equipo!

---