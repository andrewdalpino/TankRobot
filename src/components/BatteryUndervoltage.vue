<template>
    <div class="modal" :class="{ 'is-active' : open }">
        <div class="modal-background"></div>
        <div class="modal-content">
            <div class="card has-text-centered">
                <div class="card-content">
                    <div class="content">
                        <span class="icon is-large">
                            <i class="fas fa-2x fa-battery-empty"></i>
                        </span>
                        <p class="is-size-5">
                            Battery undervoltage detected. Please recharge or insert new power cells.
                        </p>
                    </div>
                </div>
                <footer class="card-footer">
                    <div class="card-footer-item">
                        <button class="button is-white" @click="open = false">
                            <span class="icon"><i class="fas fa-times"></i></span>
                            <span>Dismiss</span>
                        </button>
                    </div>
                </footer>
            </div>
        </div>
    </div>
</template>

<script>
    import bus from '../bus';

    const VIBRATE_PATTERN = [100, 30, 100];

    export default {
        data() {
            return {
                open: false,
            };
        },
        mounted() {
            bus.$on('battery-undervoltage', (payload) => {
                if (!this.open) {
                    this.open = true;

                    document.getElementById('plucky').play();

                    window.navigator.vibrate(VIBRATE_PATTERN);
                }
            });
        }
    }
</script>
