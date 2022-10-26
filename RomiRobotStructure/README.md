# Basic Robot Structure
For the [Basic Robot Structure](https://2928-frc-programmer-training.readthedocs.io/en/latest/Romi/SC/romiStructure/) section of the training guide the purpose is to explain how an FRC robot program is structured, and to introduce the Drivetrain class.   Changes made to this project are:

- Moved values `kCountsPerRevolution` and `kWheelDiameterInch` into the Constants file.
- Switched distance values from inches to meters.
- Renamed the `m_controller` variable to `m_joystick`.

### Move constants to Constants File
The Constants file is where we keep variables that don't change during the execution of the program.  There are two such variable in the *Drivetrain* class of the *RomiReference* program that really should be in the Constants file.  You'll find these variables on lines 15 and 16:

    private static final double kCountsPerRevolution = 1440.0;
    private static final double kWheelDiameterInch = 2.75591; // 70 mm

Move the two variables into the Constants file placing them between the two brackets. Notice that the are four keywords before these two variables:

- The [private](https://www.w3schools.com/java/ref_keyword_private.asp) keyword makes the variable accessible only within the declared class.  Since we want to make them accessible by any other classes, we're going to change the keyword to [public](https://www.w3schools.com/java/ref_keyword_public.asp).

-  The [static](https://www.w3schools.com/java/ref_keyword_static.asp) keyword creates attributes that can be accessed without creating an object of a class.  

- The [final](https://www.w3schools.com/java/ref_keyword_final.asp) keyword in Java means that the variable doesn't change it's value throughout the running of the program.

- [double](https://www.w3schools.com/java/ref_keyword_double.asp) is the data type of the variable that can store large fractional numbers.

In the next lab we're going to change from inches to meters so change the name of the variable `kWheelDiameterInch` to `kWheelDiameterMeters`.  Also change the value from `2.75591` to `0.07`.