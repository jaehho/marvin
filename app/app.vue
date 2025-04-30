<script></script>

<template>
  <header>
    <nav>
      <ul>
        <li>
          <NuxtLink to="/">Home</NuxtLink>
        </li>
        <li>
          <NuxtLink to="/control">Control</NuxtLink>
        </li>
        <li>
          <NuxtLink to="/test">Test</NuxtLink>
        </li>
      </ul>
    </nav>
  </header>
  <div>
    <NuxtPage />
  </div>
</template>

<style>
header {
  background-color: #ff9191;
  color: white;
  padding: 1rem;
}

nav ul {
  display: flex;
  justify-content: space-around;
  list-style: none;
}

nav ul li {
  margin: 0 1rem;
}
</style>
