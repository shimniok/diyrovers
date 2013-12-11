/** SimpleFilter implements a simple low pass integer "leaky integrator" described here:
 * 
 * http://ece124web.groups.et.byu.net/references/readings/Simple%20Software%20Lowpass%20Filter.pdf
 *
 * Well suited for filtering ADC integer values very quickly
 *
 * Michael Shimniok http://bot-thoughts.com/
 */
class SimpleFilter {
public:
    /** Creates a new filter object
     *
     * @param shift: the number of shifts to perform at each filtering input step; lower means higher bandwidth
     */
    SimpleFilter(short shift);

    /** Supplies input to the filter and returns filtered output value
     *
     * @param value is the input value to the filter, e.g., some measurement
     * @returns the filtered output value
     */
    short filter(short value);

    /** Read the current value in the filter
     *
     * @returns the current value in the filter
     */
    short value(void);

    /** Shorthand operator for value()
     *
     * @returns the current value in the filter
     */
    operator short() { return value(); }

private:
    long _filter_value;
    short _shift;
};