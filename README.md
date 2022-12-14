# Smart-Safe-Project
Hello and welcome to my smart safe project, this project was done in an Embedded systems course in college (optional course).

Describtion: As the name Suggests, I created a prototype of a smart safe which is similar to an ordinary safe that locks & unlocks as needed, but with some smart features:
- The user can connect to the safe via Bluetooth.
- Reset and change code on the Bluetooth device.
- Get a One-Use code to the bluetooth device.
- Get a warning message after 3 fault attempts to the bluetooth device.
- Access the safe with RFID card.
- Automatically close & lock the door if the user forgot to.


The following image shows the physical structure of the safe:

![WhatsApp Image 2022-12-13 at 10 44 44](https://user-images.githubusercontent.com/116517151/207268804-8b68c554-f292-4458-a1db-036dedd3766c.jpeg)

As an electronic engineer I had to design a schematic sketch of the hardware components listed and the connections required:
- Nuvoton board.
- UART interface (Bluetooth).
- Ultrasoud sensor.
- Servomotor.
- RFID reader.


The following image shows the connections:

![WhatsApp Image 2022-12-13 at 11 31 51](https://user-images.githubusercontent.com/116517151/207280579-77aa0c7e-18bb-4cc3-9c5c-86a62a94208f.jpeg)


In order to program the microcontoller to satisfy the desired functions I used µVision software and arm keil Embedded development tool, wrote a code in C language and edited the component's driver. (code attached)

Please feel free to visit this link to my google drive where I recorded a video of the safe fully functioning: 
- https://drive.google.com/drive/folders/19ciINSq1C8gJDozYlxyJUooAQ1akTDUl?usp=sharing
