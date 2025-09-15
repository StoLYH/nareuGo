import apiClient from './client'                                                                                                                                                  

export const authAPI = {
// 사용자 정보 조회                                                                                                                                                             
getUserInfo: () => apiClient.get('/api/v1/auth/user'),

// 토큰 갱신                                                                                                                                                                    
refreshToken: () => apiClient.post('/api/v1/auth/refresh'),

// 로그아웃                                                                                                                                                                     
logout: () => apiClient.post('/api/v1/auth/logout'),

// Access Token으로 사용자 정보 조회 (OAuth 콜백 후)                                                                                                                            
getUserByToken: (token) =>                                                                                                                                                      
    apiClient.get('/api/v1/auth/user', {
    headers: {
        Authorization: `Bearer ${token}`                                                                                                                                          
    }
    })
}