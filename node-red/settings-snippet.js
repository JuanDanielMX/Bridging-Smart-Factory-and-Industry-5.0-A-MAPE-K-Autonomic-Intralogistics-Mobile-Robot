// Merge this property into ~/.node-red/settings.js.
// Node.js provides crypto; it is not an additional npm dependency.
functionGlobalContext: {
    crypto: require("crypto")
},
