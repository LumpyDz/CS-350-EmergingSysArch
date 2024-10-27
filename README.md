# CS-350-EmergingSysArch
Module 8 Journal - 
10/27/2024 - 
Grant Sorenson

I enjoyed this course and I thought it taught me a lot about embedded software and working with embedded devices. Two of the projects I will cover in this journal include the “gpio” and “uart” projects. The gpio project taught me about how to manipulate the electrical signals outputting to LEDs. This project was simple, but it was important to read the documentation and refresh my memory on the C programming language because the libraries used were unfamiliar to me. I think I could have improved my documentation of the project, including more information as to why certain things functioned the way they did like the button callback functions. Some of the tools and resources I gained include learning how to create embedded software with code composer and setting up an embedded environment connected to hardware. Some of the skills I learned in this course may not entirely relate to what I want to do as a career, but one that will help is learning about the logic of a tasking system based on a single timer. Finding the lowest common denominator and using Enums for states is a great way to organize a tasking system. I tried to make this project maintainable and readable by getting rid of unnecessary code and including documentation so a developer would know what was happening and how the device functions. 

The uart project was different from the gpio project because it required me to use a different piece of software that was included with the embedded device. Instead of using a library that connected the LED pins, we now use a library that creates a server and can handle outputting responses to a communication port. This solved the problem of grabbing results from an embedded device and sending them to an analysis program that parses that data for some specific task. This project was difficult because I had some initial issues with my board, but after I got through them I had a much easier time configuring the uart driver and utilizing the library to make text echo to and from the com port. One of the things I could have improved was my bug testing while making the project. I discovered afterward I had a bug in which text would become garbled and incorrect, so I fixed it before submitting it to his repository. A few tools and resources this allows me to add to my network include working with servers in an embedded environment and device communication in an embedded environment. Some skills I will take with me from this project include the ability to learn a new library quickly, and read through code understanding what its doing and what I will need to do to change it. I made this project maintainable and readable by including documentation and  
