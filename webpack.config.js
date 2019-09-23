const VueLoaderPlugin = require('vue-loader/lib/plugin');
const MiniCssExtractPlugin = require('mini-css-extract-plugin');
const TerserJSPlugin = require('terser-webpack-plugin');
const OptimizeCSSAssetsPlugin = require('optimize-css-assets-webpack-plugin');
const path = require('path');

module.exports = {
    mode: process.env.NODE_ENV,
    entry: {
        app: './src/app.js',
    },
    output: {
        path: path.resolve(__dirname, 'data'),
        publicPath: '/',
        filename: 'app.js',
    },
    module: {
        rules: [
            { test: /\.js$/, loader: 'babel-loader' },
            { test: /\.vue$/, loader: 'vue-loader' },
            { test: /\.css$/, use: [
                process.env.NODE_ENV === 'development'
                  ? 'vue-style-loader'
                  : MiniCssExtractPlugin.loader,
                'css-loader',
            ]},
            { test: /\.(scss|sass)$/, use: [
                MiniCssExtractPlugin.loader,
                {
                    loader: 'css-loader',
                },
                {
                    loader: 'sass-loader',
                    options: {
                        sourceMap: process.env.NODE_ENV === 'development',
                    },
                },
            ]},
            { test: /\.(woff|woff2)$/, use: [
                {
                    loader: 'file-loader',
                    options: {
                        name: '[name].[ext]',
                    },
                },
            ]},
        ],
    },
    plugins: [
        new VueLoaderPlugin(),
        new MiniCssExtractPlugin('app.css'),
    ],
    optimization: {
        minimizer: [
            new TerserJSPlugin({}),
            new OptimizeCSSAssetsPlugin({}),
        ],
    },
    resolve: {
        alias: { vue: 'vue/dist/vue.esm.js' },
    },
};