# FlightController

Figured I should design a PCB and a flight controller seemed pretty approachable compared to how complicated PC's are getting. DroneMesh designed a F4 flight controller that uses essentially three chips. The Gyro MPU6000, the processor STM32F4xx, and one diode to feed the 3.3v regulator from USB or external 5v. I would like to make a F7 flight controller and would like for it to be similar to the JBF7_V2. I am currently using Altium Designer for schematic and layout (get a free license due to my masters degreee) but I will likely make the files available in as many formats as I possibly can (KiCad, Cadence, EasyEDA, DipTrace, others?)

For the most part the schematic is the most useful, as I find this is really an exercise in picking chips and connecting nets together, layout can be automated quite a bit, but I'm sure manual part placement will occur as well as setting the 30x30 size and 4mm mounting holes.

			
		"Special Pins" have timers, like motor outputs, camera control, LED_STRIP, etc

		

For manufacturing I'll probably use PCBWay or JLCPCB


Good video/channel on PCB design:

https://www.youtube.com/watch?v=aVUqaB0IMh4

Links to drone meshes stuff:

https://oshwlab.com/mesh.drone/mesh-fc-v1-backup

https://www.pcbway.com/project/member/?bmbno=82F43017-E23A-4A

https://www.youtube.com/c/DroneMesh


Version with OSD chip:

https://easyeda.com/modules/DroneMesh-OpenFC-Schematic_02d7d85512334af9a66e7b7c09a951f3
