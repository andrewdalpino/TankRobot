import Plotly from 'plotly.js/src/core';
import Scatter from 'plotly.js/src/traces/scatter';
import Pie from 'plotly.js/src/traces/pie';

Plotly.register([
    Scatter, Pie,
]);

export default Plotly;
