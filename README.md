# QUICKSILVER [![Build Status](https://ci.bkleiner.codes/api/badges/NotFastEnuf/Guano/status.svg)](https://ci.bkleiner.codes/NotFastEnuf/Guano)
Flight Controller Firmware based on Silverware by Silver13

**NFE NOTES:**

This is the GCC dev branch of QUICKSILVER using STM's CUBE IDE

-----

**WIKI NOTES** add details whenever, will be compiled before relese and added as a proper wiki page

1.  The aim of the project

2.  Key features

3.  Getting Quicksilver

4.  The configurator

5.  Initial setup

6.  More detail

   * pids
   * filters
   * motor test
   * esc settings
   * aux switches
   * osd
   * inflight data
  
7.  Credits

8.  NFE details (Copy from RCG post)
>>Sir D asked the most important question. What do these mixers actually do. Let's start with that.
To summarize their function the code ... Let's first look at a system without them (which is where silverware started). The flow of logic goes like this. Flight mode algorithm (level or acro) is first established and determines the math that governs how setpoint and error are handled using stick input and gyro data. That data is then fed into the pid controller which establishes gains for all the motor commands but the data is output in a axis by axis basis. Next that information goes into what I would call stage 1 of a mixer which is based on the physical layout of your craft. We only support quad with motors on corners (although I have helped a few guys do a custom plus layout... But never pushed the code into the fork). Here is where commands are assigned to motors by adding them together and assigning a positive or negative value.
For example, throttle is always positive, a front right and left rear motor may have positive values for yaw while the front left and right rear would be assigned negative...etc.
Now we have commands for each motor that can be translated into pwm output or dshot or whatever.
BUT keep in mind that a motor has an upper and lower limit (1 and 0).... So the last thing that happens is that any values above or below the limits of 1 or 0 have to get cut off. This is called clipping. Any time clipping happens, the precise math performed by the flight algorithm and divided out on a motor per motor basis is now an inaccurate output. Like full throttle doesn't go straight up, or a dive with throttle at 5 percent bobbles around on the way down. Or a roll at low throttle happens slower than a roll at half throttle.

Ok, with me so far.... so over time silver gave us some tools to deal with clipping and the negative effects it has on consistent flight performance. I am going to discuss each one.

First is the pidsum and integral sum limits themselves which cap the values being output by the system before the mixer. But this is just the same as clipping just pulled out into each axis being adjustable and has a really non-intuitive link to the final motor outputs. Many of you have often messed with these values to patch up flight performance. I would argue that these limits are best set high enough to never clip pid output during flight with one exception ... A collision or a crash. We need the pid controller to compute the right math... Except during a crash when it could go nuts trying to stabilize craft that it can't stabilize and values get insane large. That's the time to clip a pidsum.

Next we have clip feed forward. This adds any clipped motor outputs to the next loop. In hopes that the data ... While a little late, may get the craft back on track a little sooner and before we notice. The problem here is that once off track due to clipping, the next motor output is also likely to clip again anyway ... In fact the motor has to return to an output where it's not clipping for this feature to do anything. By this time the pid loop is already aware of that error.... And we just added to it and added to it late. If your craft is well tuned ... It's gonna overshoot and oscillate. If you've seen it work well without this oscillation you either were tuned low, or weren't pushing your flight envelope into clipping. I would argue this feature can be totally removed.

Then we have mix increase mix lower throttle. The approach here is to scale back any command that goes over 1 or under zero up to a user set limit. What's cool about this is that there is some filter delay added to the response so that the scaling correction to the pid controller isn't leading motor response too much and it's more in sync with slower respose time motors like brushed. So the scaling gets delayed and clipped beyond a certain point. The mix lower works great on brushed motors and as long as the motors themselves are in great shape we can clip beyond 10 percent scaling and gain a good chunk of extra thrust for little downside on brushed motors. The mix increase is essentially just betaflight airmode logic but with an adjustment limit and a delay. It doess not respond fast enough for low throttle dives and bobbles are likely to occur on all crafts. Mix increase 3 works much better for all motor types. The last thing to address is what happens when motors are both above and below the 1 and 0 limits. In this case the output of the system is very dependent on which function comes first in control.c or what the user set limits are. That's not exactly awesome or intuitive.

Another option for scaling vs clipping is the mix 3 features. There is no delay and full adjustments to scaling are made within the same pid loop up to the user set limit. This approach is best for motors that have fast response times like brushless. Mix increase 3 is the same as betaflight airmode but with an adjustable limit on it's response. Mix lower 3 is the same scaling on the top side again with a limit. Once again if one motor is commanded over 1 and one below 0, the output can depend on which function comes first and what your limits are. This again can create some unpredictable results and was a scenario that was not anticipated in the code logic.

Then there is the betaflight mixer or the joelucid mixer whatever you prefer to call it. It's the same as mix 3 but has no limits so it operates at full strength and it also addresses the troublesome case where one motor is below 0 and one above 1. At all times this mixer will retain the exact proportional difference in the math established by the pid controller and keep all motor outputs between 0 and 1. This is mathmatically the best possible approach if there isn't a response delay to be considered like in a brushed motor.

I propose there are two best case scenarios and only two mixers we need.
1. Brushed:. Mix increase 3 logic and mix lower throttle logic paired together. Adjustable limits in place on both the top end and the bottom end.
2. Brushless: Betaflight mixer logic which
is the exact same as mix 3 but addressess that pesky third case of motor outputs, and with the addition of adjustable limits from the mix 3 function. So that if you feel airmode behavior is too exaggerated you can turn down it's response(even as far as totally off if you prefer an "idle up" approach to low throttle behavior) and if you feel too much power is robbed you can also limit the scaling down by adding some clipping back in. Simple - one brushless mixer with 2 adjustments and no more duplicate logic making a mess of code. Scale as much as you want to retain the math from the pid controller or clip as much as you want if you feel some of it's decisions can be ignored based on your flying style or craft details.

Now we aren't done yet cause we have an enable motor minimum feature. So we started with outputs to go beyond full on and full off based on exact math from the pid controller, gyro, and sticks. We then decided how much of this exact math we want to keep by squishing it down or just ignoring some of it to any motor that is beyond the physical limit of on or off. Now we have yet another source of clipping in motor min enable. It just tossed out any values below the user set value once again adding error to the system. This feature should also scale and I would argue could do it right inside of the brushless mixer so that user set limits apply it it as well. Brushed has little need for this but brushless needs this to prevent desyncs at low command values. Again... I say we have the opportunity for cleaner and more accurate code that addresses all possible user needs.

Think we're done yet... Nope. Now we have motor outputs that fit between on and off. And here is where we apply motor curves or thrust linearization. Pid controllers are designed for linear systems. Motor outputs are not linear as a function of pwm (or dshot) output. Approaches here get touchy and really depend on your origional tuning style. Say you tune at the limits of extreme performance. You need to compensate the the lesser motor outputs according to their relative performance in other ranges TO the maximum outputs. Or say you are well tuned to a mid-range motor output. Well now you need to compensate the upper limits TO the mid-range. Now it gets even trickier cause flight is more often than not a combination of many outputs ... So how can you tell the difference for what response needs to be linearized and how much. Every prop motor combination is going to be different and that is a huge can of worms. And if you don't really understand what your algorithms are doing - tuning can be pretty tricky.

I could go on and on about this stuff and didn't even scratch on torque boost, d term setpoint weight, throttle boost as it all plays a part in how much clipping (broken pid math) happens in system too. Maybe that's enough for now.

This will obviously need editing :)



