# Lab 6: PID-Controller

Due: **Friday 3/15/19 6:09 pm**

## Description

This is a PID controller lab for lateral control. You will work alone to code up
a PID controller to perform lateral control of a simulated car moving at
constant speed whose motion is governed by the bicycle kinematic model. You will
also learn how to implement a PID auto-tuning algorithm called twiddle. The jupyter notebook is called 
`PID-Controller.ipynb`.

Do all the TODO's in all of the code blocks.

For implementing the auto-tuning algorithm in the final code block, you will all
have your own assigned noise/drift parameters. Make sure to set the values
`drift_angle`, `steering_noise`, and `distance_noise` to your assigned values in
your final submission. Failure to do so, will result will result in a 10%
deduction on your overall grade. This is to ensure you are all tuning a 
controller for your own "car."

You may find your assigned values `drift_angle`, `steering_noise`, and
`distance_noise`, in that order, below:

<pre>
Calvin  :    10.0, 0.10, 0.10
Daniel  :    25.0, 0.05, 0.10
Faraz   :   -15.0, 0.10, 0.05
Frank   :   -20.0, 0.02, 0.02
Griffin :   -10.0, 0.15, 0.05
Jeff    :    15.0, 0.05, 0.15
Kolin   :    5.00, 0.05, 0.25
Victoria:    15.0, 0.05, 0.10
Wenda   :   -10.0, 0.10, 0.10
Toshi   :    25.0, 0.15, 0.05
</pre>

Try your best to minimize overshoot, ringing, provide a reasonble setting time,
etc. You will probably find that using the `run()` function with the twiddle
algorithm does not result in a desirable car ride. This has much to do with the
fact that the cost function defined in the `run()` function does not try to
achieve any sort of desirable rise time, setting time, etc. It just tries to
minimize the average cross-track error.

While not required, extra credit will be given to anyone who defines their own
unique cost function that is able to tune the controller in more useful
ways...Feel free to modify the twiddle algorithm as needed as well.

## Submission
You may submit the jupyter notebook as is or as a zip file on canvas.
