typedef unsigned int tick;
typedef int value;
typedef char flag;

/** Simple library for scheduling events by polling time, that is, avoiding interrupts.
 *  Can schedule up to 64 events.  The schedule is defined by the number of ticks making
 *  up the initial period of the schedule, and either a start and end value or values
 *  at specific tick values.
 */
class Schedule {
    public:
        /** Sets the behavior of getNext() after the initial schedule period has exceeded.
         *
         * @param repeat means the schedule values are repeated
         * @param hold means the last schedule value is provided after the schdule is done
         */
        enum { repeat=0x02, hold=0x04 };

        /** Creates an empty schedule
         */
        Schedule();
        
        /** Creates a schedule based on a linear function. See set()
         *
         *  @param scale
         *  @param max the maximum tick value of the clock, sets the period
         *  @param start is the value returned at tick == 0 
         *  @param stop is the value returned at tick == ticks-1
         *  @param m selects the mode / behavior of the schedule when getNext() called after period exceeded
         */
        Schedule(unsigned int scale, tick max, value start, value stop, flag m);
            
        /** Sets a ratio of time to ticks. See clockTicked()
         *
         * @param timePerTick specifies the number of time units per tick
         */
        void scale(unsigned int scale);
        
        /** Sets the total number of ticks to run the loop
         */
        void max(tick max);
        
        /** Sets behavior of getNext() when called after tickCount exceeded
         */
        void mode(flag m);
        
        /** sets the value at the specified tick
         *
         * @param t specifies the scheduled tick
         * @param v specifies the value to return when tick==whichTick
         */        
        void set(tick t, value v);
        
        /** Set schedule based on a linear function
         *
         *  @param ticks total number of ticks over which the schedule is valid
         *  @param startValue is the value returned at tick == 0 
         *  @param stopValue is the value returned at tick == ticks-1
         */
        void set(unsigned int scale, tick max, value start, value stop, flag m); 
        
        /** get the next value for schedule's current time.  Use with ticked()
         *
         * @returns the value at the current schedule's time
         */
        value get();
        
        /** increment the clock and get the next value in the schedule
         *
         * @returns the value at the schedule's next clock tick
         */
         value next();
         
        /** Pass in some unit of time and determine if the 'clock' has ticked.
         *  Suppose timePerTick == 20 and you pass in the elapsed time in milliseconds
         *  then this function returns true every 20ms.
         *
         *  @param time the integer corresponding to elapsed time
         *  @returns true if the elapsed time % timePerTick == 0
         */
        bool ticked(unsigned int time);
        
        /** Are we done with the schedule?
         *
         * @returns true if schedule is done; see max()
         */
        bool done();

    private:
        unsigned int _scale;
        tick _max;
        tick _clock;
        flag _mode;
        int _schedule[64];
        bool _validTick(tick t);
};
        