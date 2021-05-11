<template>
    <div>
        <main-nav></main-nav>
        <main>
            <router-view></router-view>
        </main>
        <rollover-detected></rollover-detected>
        <communication-error></communication-error>
        <audio id="plucky" src="./sounds/plucky.ogg" crossOrigin="anonymous"></audio>
    </div>
</template>
