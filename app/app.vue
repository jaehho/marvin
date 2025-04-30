<script>
export default {
  data() {
    return {
      pages: [
        { name: 'Home', path: '/' },
        { name: 'Pose', path: '/simple_pose' },
        { name: 'Test', path: '/test' },
      ]
    }
  }
}
</script>

<template>
  <header>
    <nav>
      <ul>
        <li v-for="page in pages" :key="page.path">
          <NuxtLink :to="page.path">{{ page.name }}</NuxtLink>
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
