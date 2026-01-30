/**
 * Smart Attendance System - Main JavaScript
 * Handles all UI interactions, API calls, and data management
 */

// ========================================
// Global Variables & Configuration
// ========================================

let currentPage = 'dashboard';
let systemData = {
    users: [],
    attendance: [],
    sessions: [],
    config: {},
    stats: {}
};

let currentEditUserId = null;
let currentSessionId = null;

const API_BASE = '';

// ========================================
// Initialization
// ========================================

document.addEventListener('DOMContentLoaded', function() {
    initializeApp();
});

async function initializeApp() {
    console.log('Initializing Smart Attendance System...');
    
    // Start time display
    updateTimeDisplay();
    setInterval(updateTimeDisplay, 1000);
    
    // Initialize navigation
    initializeNavigation();
    
    // Load initial data
    await loadSystemData();
    
    // Setup form handlers
    setupFormHandlers();
    
    // Hide loading overlay
    setTimeout(() => {
        document.getElementById('loadingOverlay').classList.add('hidden');
    }, 500);
    
    // Auto-refresh dashboard every 10 seconds
    setInterval(() => {
        if (currentPage === 'dashboard') {
            refreshDashboard();
        }
    }, 10000);
}

// ========================================
// Navigation
// ========================================

function initializeNavigation() {
    const navLinks = document.querySelectorAll('.nav-link');
    
    navLinks.forEach(link => {
        link.addEventListener('click', function(e) {
            e.preventDefault();
            const page = this.dataset.page;
            navigateToPage(page);
        });
    });
}

function navigateToPage(pageName) {
    // Update nav links
    document.querySelectorAll('.nav-link').forEach(link => {
        link.classList.remove('active');
        if (link.dataset.page === pageName) {
            link.classList.add('active');
        }
    });
    
    // Update pages
    document.querySelectorAll('.page').forEach(page => {
        page.classList.remove('active');
    });
    
    const targetPage = document.getElementById(`${pageName}-page`);
    if (targetPage) {
        targetPage.classList.add('active');
        currentPage = pageName;
        
        // Load page-specific data
        loadPageData(pageName);
    }
}

async function loadPageData(pageName) {
    switch(pageName) {
        case 'dashboard':
            await refreshDashboard();
            break;
        case 'users':
            await loadUsers();
            break;
        case 'attendance':
            await loadAttendance();
            break;
        case 'session':
            await loadSessionControl();
            break;
        case 'settings':
            await loadSettings();
            break;
    }
}

// ========================================
// API Calls
// ========================================

async function apiCall(endpoint, method = 'GET', data = null) {
    try {
        const options = {
            method: method,
            headers: {
                'Content-Type': 'application/json'
            }
        };
        
        if (data && method !== 'GET') {
            options.body = JSON.stringify(data);
        }
        
        const response = await fetch(API_BASE + endpoint, options);
        
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }
        
        const contentType = response.headers.get('content-type');
        if (contentType && contentType.includes('application/json')) {
            return await response.json();
        }
        
        return await response.text();
        
    } catch (error) {
        console.error('API Error:', error);
        showToast('Connection error: ' + error.message, 'error');
        return null;
    }
}

async function loadSystemData() {
    try {
        const data = await apiCall('/api/system/data');
        if (data) {
            systemData = data;
            updateDashboardStats();
        }
    } catch (error) {
        console.error('Failed to load system data:', error);
    }
}

// ========================================
// Dashboard Functions
// ========================================

async function refreshDashboard() {
    await loadSystemData();
    await loadRecentAttendance();
    await loadSessionStatus();
}

function updateDashboardStats() {
    const stats = systemData.stats || {};
    
    document.getElementById('totalUsers').textContent = stats.total_users || 0;
    document.getElementById('totalStudents').textContent = stats.total_students || 0;
    document.getElementById('totalFaculty').textContent = stats.total_faculty || 0;
    document.getElementById('todayPresent').textContent = stats.today_attendance || 0;
}

async function loadRecentAttendance() {
    const tbody = document.getElementById('recentTableBody');
    const attendance = systemData.attendance || [];
    
    // Get last 10 records
    const recent = attendance.slice(-10).reverse();
    
    tbody.innerHTML = '';
    
    if (recent.length === 0) {
        tbody.innerHTML = `
            <tr>
                <td colspan="6" class="text-center">
                    <div class="empty-state">
                        <div class="empty-state-icon">📋</div>
                        <div class="empty-state-text">No attendance records yet</div>
                    </div>
                </td>
            </tr>
        `;
        return;
    }
    
    recent.forEach(record => {
        const user = systemData.users.find(u => u.user_id === record.user_id);
        if (!user) return;
        
        const time = new Date(record.timestamp * 1000);
        const timeStr = time.toLocaleTimeString('en-US', { 
            hour: '2-digit', 
            minute: '2-digit' 
        });
        
        const statusClass = getStatusClass(record.status);
        const methodBadge = record.auth_method === 0 ? 
            '<span class="badge badge-info">👆 FP</span>' : 
            '<span class="badge badge-success">💳 RFID</span>';
        
        const row = `
            <tr>
                <td>${timeStr}</td>
                <td>${escapeHtml(user.name)}</td>
                <td>${escapeHtml(user.roll_number || user.user_id)}</td>
                <td>${getUserTypeLabel(user.type)}</td>
                <td class="${statusClass}">${getStatusLabel(record.status)}</td>
                <td>${methodBadge}</td>
            </tr>
        `;
        
        tbody.innerHTML += row;
    });
}

async function loadSessionStatus() {
    const container = document.getElementById('sessionStatus');
    const session = systemData.current_session;
    
    if (session && session.active) {
        const startTime = new Date(session.start_time * 1000);
        const timeStr = startTime.toLocaleString('en-US', {
            year: 'numeric',
            month: 'short',
            day: 'numeric',
            hour: '2-digit',
            minute: '2-digit'
        });
        
        container.innerHTML = `
            <div class="session-active">
                <h3 style="color: var(--success-color); margin-bottom: 1rem;">
                    ✓ Session Active
                </h3>
                <div class="session-info">
                    <div class="session-info-item">
                        <span class="session-info-label">Subject:</span>
                        <span class="session-info-value">${escapeHtml(session.subject_name)}</span>
                    </div>
                    <div class="session-info-item">
                        <span class="session-info-label">Faculty:</span>
                        <span class="session-info-value">${escapeHtml(session.faculty_name)}</span>
                    </div>
                    <div class="session-info-item">
                        <span class="session-info-label">Started:</span>
                        <span class="session-info-value">${timeStr}</span>
                    </div>
                    <div class="session-info-item">
                        <span class="session-info-label">Present:</span>
                        <span class="session-info-value" style="color: var(--success-color); font-weight: 600;">
                            ${session.total_present || 0}
                        </span>
                    </div>
                    <div class="session-info-item">
                        <span class="session-info-label">Absent:</span>
                        <span class="session-info-value" style="color: var(--danger-color); font-weight: 600;">
                            ${session.total_absent || 0}
                        </span>
                    </div>
                </div>
            </div>
        `;
    } else {
        container.innerHTML = `
            <div class="session-inactive">
                <h3 style="color: var(--text-secondary); margin-bottom: 0.5rem;">
                    ✗ No Active Session
                </h3>
                <p>Start a new session to begin taking attendance</p>
                <button class="btn btn-primary" style="margin-top: 1rem;" onclick="navigateToPage('session')">
                    Start Session
                </button>
            </div>
        `;
    }
}

function refreshSession() {
    showToast('Refreshing session data...', 'info');
    loadSessionStatus();
}

function filterRecent() {
    const filter = document.getElementById('recentFilter').value;
    const tbody = document.getElementById('recentTableBody');
    const rows = tbody.getElementsByTagName('tr');
    
    for (let row of rows) {
        if (filter === 'all') {
            row.style.display = '';
        } else {
            const typeCell = row.cells[3];
            if (typeCell) {
                const type = typeCell.textContent.toLowerCase();
                row.style.display = type.includes(filter) ? '' : 'none';
            }
        }
    }
}

// ========================================
// Users Management
// ========================================

async function loadUsers() {
    await populateDepartmentFilters();
    await displayUsers();
}

async function displayUsers() {
    const tbody = document.getElementById('usersTableBody');
    const users = systemData.users || [];
    
    tbody.innerHTML = '';
    
    if (users.length === 0) {
        tbody.innerHTML = `
            <tr>
                <td colspan="10" class="text-center">
                    <div class="empty-state">
                        <div class="empty-state-icon">👥</div>
                        <div class="empty-state-text">No users found</div>
                        <div class="empty-state-subtext">Click "Add New User" to get started</div>
                    </div>
                </td>
            </tr>
        `;
        return;
    }
    
    users.forEach(user => {
        const attendancePercent = calculateAttendancePercentage(user);
        const fpStatus = user.fingerprint_enrolled ? 
            '<span style="color: var(--success-color);">✓</span>' : 
            '<span style="color: var(--danger-color);">✗</span>';
        const rfidStatus = user.rfid_enrolled ? 
            '<span style="color: var(--success-color);">✓</span>' : 
            '<span style="color: var(--danger-color);">✗</span>';
        
        const row = `
            <tr>
                <td>${user.user_id}</td>
                <td>${escapeHtml(user.name)}</td>
                <td>${getUserTypeLabel(user.type)}</td>
                <td>${escapeHtml(user.roll_number || '-')}</td>
                <td>${escapeHtml(user.department || '-')}</td>
                <td>${escapeHtml(user.email || '-')}</td>
                <td class="text-center">${fpStatus}</td>
                <td class="text-center">${rfidStatus}</td>
                <td>${attendancePercent}%</td>
                <td>
                    <div class="action-buttons">
                        <button class="action-btn action-btn-view" onclick="viewUser(${user.user_id})" title="View">
                            👁️
                        </button>
                        <button class="action-btn action-btn-edit" onclick="editUser(${user.user_id})" title="Edit">
                            ✏️
                        </button>
                        <button class="action-btn action-btn-delete" onclick="confirmDeleteUser(${user.user_id})" title="Delete">
                            🗑️
                        </button>
                    </div>
                </td>
            </tr>
        `;
        
        tbody.innerHTML += row;
    });
}

function searchUsers() {
    const searchTerm = document.getElementById('userSearch').value.toLowerCase();
    const tbody = document.getElementById('usersTableBody');
    const rows = tbody.getElementsByTagName('tr');
    
    for (let row of rows) {
        const text = row.textContent.toLowerCase();
        row.style.display = text.includes(searchTerm) ? '' : 'none';
    }
}

function filterUsers() {
    const typeFilter = document.getElementById('userTypeFilter').value;
    const deptFilter = document.getElementById('departmentFilter').value;
    const statusFilter = document.getElementById('statusFilter').value;
    
    const tbody = document.getElementById('usersTableBody');
    const rows = tbody.getElementsByTagName('tr');
    
    for (let row of rows) {
        let showRow = true;
        
        // Type filter
        if (typeFilter !== 'all') {
            const typeCell = row.cells[2];
            if (typeCell && !typeCell.textContent.toLowerCase().includes(typeFilter)) {
                showRow = false;
            }
        }
        
        // Department filter
        if (deptFilter !== 'all') {
            const deptCell = row.cells[4];
            if (deptCell && deptCell.textContent !== deptFilter) {
                showRow = false;
            }
        }
        
        // Status filter
        if (statusFilter !== 'all') {
            // Status filtering logic here
        }
        
        row.style.display = showRow ? '' : 'none';
    }
}

function showAddUserModal() {
    document.getElementById('userModalTitle').textContent = 'Add New User';
    document.getElementById('userForm').reset();
    currentEditUserId = null;
    
    document.getElementById('fpStatus').textContent = 'ENROLL AFTER SAVE';
    document.getElementById('fpStatus').className = 'badge badge-danger';
    document.getElementById('rfidStatus').textContent = 'ENROLL AFTER SAVE';
    document.getElementById('rfidStatus').className = 'badge badge-danger';
    
    document.getElementById('userModal').classList.add('active');
}

function closeUserModal() {
    document.getElementById('userModal').classList.remove('active');
}

function editUser(userId) {
    const user = systemData.users.find(u => u.user_id === userId);
    if (!user) return;
    
    currentEditUserId = userId;
    document.getElementById('userModalTitle').textContent = 'Edit User';
    
    // Populate form
    document.getElementById('userName').value = user.name;
    document.getElementById('userType').value = getUserTypeValue(user.type);
    document.getElementById('userRollNumber').value = user.roll_number || '';
    document.getElementById('userEmail').value = user.email || '';
    document.getElementById('userDepartment').value = user.department || '';
    document.getElementById('userSemester').value = user.semester || '1';
    document.getElementById('userStatus').value = getUserStatusValue(user.status);
    
    // Update enrollment status
    if (user.fingerprint_enrolled) {
        document.getElementById('fpStatus').textContent = 'Enrolled';
        document.getElementById('fpStatus').className = 'badge badge-success';
    }
    
    if (user.rfid_enrolled) {
        document.getElementById('rfidStatus').textContent = 'Registered';
        document.getElementById('rfidStatus').className = 'badge badge-success';
    }
    
    toggleStudentFields();
    document.getElementById('userModal').classList.add('active');
}

function viewUser(userId) {
    const user = systemData.users.find(u => u.user_id === userId);
    if (!user) return;
    
    // In a real implementation, show detailed view modal
    showToast(`Viewing details for ${user.name}`, 'info');
}

function confirmDeleteUser(userId) {
    const user = systemData.users.find(u => u.user_id === userId);
    if (!user) return;
    
    if (confirm(`Are you sure you want to delete ${user.name}?`)) {
        deleteUser(userId);
    }
}

async function deleteUser(userId) {
    try {
        const result = await apiCall(`/api/user/delete`, 'POST', { user_id: userId });
        
        if (result && result.success) {
            // Remove from local data
            systemData.users = systemData.users.filter(u => u.user_id !== userId);
            
            showToast('User deleted successfully', 'success');
            await displayUsers();
            await updateDashboardStats();
        } else {
            showToast('Failed to delete user', 'error');
        }
    } catch (error) {
        console.error('Delete user error:', error);
        showToast('Error deleting user', 'error');
    }
}

function toggleStudentFields() {
    const userType = document.getElementById('userType').value;
    const studentFields = document.getElementById('studentFields');
    const rollNumberGroup = document.getElementById('rollNumberGroup');
    
    if (userType === 'student') {
        studentFields.style.display = 'flex';
        rollNumberGroup.style.display = 'block';
    } else {
        studentFields.style.display = 'none';
        rollNumberGroup.style.display = 'none';
    }
}

async function enrollFingerprint() {

    if (!currentEditUserId || !document.getElementById('userName').value) {
        showToast('Please fill in user details first', 'warning');
        return;
    }
    
    showToast('Place finger on scanner...', 'info');
    
    try {
        const result = await apiCall('/api/fingerprint/enroll', 'POST', {
            user_id: currentEditUserId
        });
        
        if (result && result.success) {
            document.getElementById('fpStatus').textContent = 'Enrolled';
            document.getElementById('fpStatus').className = 'badge badge-success';
            showToast('Fingerprint enrolled successfully!', 'success');
        } else {
            showToast('Fingerprint enrollment failed', result.status.message);
        }
    } catch (error) {
        console.error('Enrollment error:', error);
        showToast('Enrollment error', 'error');
    }
}

async function enrollRFID() {
    if (!currentEditUserId && !document.getElementById('userName').value) {
        showToast('Please fill in user details first', 'warning');
        return;
    }
    
    showToast('Tap RFID card on reader...', 'info');
    
    try {
        const result = await apiCall('/api/rfid/register', 'POST', {
            user_id: currentEditUserId
        });
        
        if (result && result.success) {
            document.getElementById('rfidStatus').textContent = 'Registered';
            document.getElementById('rfidStatus').className = 'badge badge-success';
            showToast('RFID card registered successfully!', 'success');
        } else {
            showToast('RFID registration failed', 'error');
        }
    } catch (error) {
        console.error('RFID registration error:', error);
        showToast('Registration error', 'error');
    }
}
// ========================================
// Attendance Management
// ========================================

async function loadAttendance() {
    await populateAttendanceDepartmentFilter();
    setDefaultDateRange();
    await displayAttendance();
    updateAttendanceStats();
}

function setDefaultDateRange() {
    const today = new Date();
    const weekAgo = new Date(today.getTime() - 7 * 24 * 60 * 60 * 1000);
    
    document.getElementById('dateFrom').valueAsDate = weekAgo;
    document.getElementById('dateTo').valueAsDate = today;
}

async function displayAttendance() {
    const tbody = document.getElementById('attendanceTableBody');
    const attendance = getFilteredAttendance();
    
    tbody.innerHTML = '';
    
    if (attendance.length === 0) {
        tbody.innerHTML = `
            <tr>
                <td colspan="8" class="text-center">
                    <div class="empty-state">
                        <div class="empty-state-icon">📋</div>
                        <div class="empty-state-text">No attendance records found</div>
                        <div class="empty-state-subtext">Try adjusting the filters</div>
                    </div>
                </td>
            </tr>
        `;
        return;
    }
    
    attendance.reverse().forEach(record => {
        const user = systemData.users.find(u => u.user_id === record.user_id);
        if (!user) return;
        
        const date = new Date(record.timestamp * 1000);
        const dateStr = date.toLocaleDateString('en-US');
        const timeStr = date.toLocaleTimeString('en-US', { 
            hour: '2-digit', 
            minute: '2-digit' 
        });
        
        const statusClass = getStatusClass(record.status);
        const methodBadge = record.auth_method === 0 ? '👆 FP' : '💳 RFID';
        
        const row = `
            <tr>
                <td>${dateStr}</td>
                <td>${timeStr}</td>
                <td>${escapeHtml(user.name)}</td>
                <td>${escapeHtml(user.roll_number || '-')}</td>
                <td>${escapeHtml(user.department || '-')}</td>
                <td>${escapeHtml(record.session_name || '-')}</td>
                <td class="${statusClass}">${getStatusLabel(record.status)}</td>
                <td>${methodBadge}</td>
            </tr>
        `;
        
        tbody.innerHTML += row;
    });
}

function getFilteredAttendance() {
    const dateFrom = document.getElementById('dateFrom').valueAsDate;
    const dateTo = document.getElementById('dateTo').valueAsDate;
    const dept = document.getElementById('attDepartmentFilter').value;
    const status = document.getElementById('attStatusFilter').value;
    
    let filtered = systemData.attendance || [];
    
    // Date filter
    if (dateFrom) {
        const fromTimestamp = Math.floor(dateFrom.getTime() / 1000);
        filtered = filtered.filter(r => r.timestamp >= fromTimestamp);
    }
    
    if (dateTo) {
        const toTimestamp = Math.floor((dateTo.getTime() + 86400000) / 1000);
        filtered = filtered.filter(r => r.timestamp < toTimestamp);
    }
    
    // Department filter
    if (dept !== 'all') {
        filtered = filtered.filter(r => {
            const user = systemData.users.find(u => u.user_id === r.user_id);
            return user && user.department === dept;
        });
    }
    
    // Status filter
    if (status !== 'all') {
        const statusValue = getStatusValue(status);
        filtered = filtered.filter(r => r.status === statusValue);
    }
    
    return filtered;
}

function filterAttendance() {
    displayAttendance();
    updateAttendanceStats();
}

function resetAttendanceFilters() {
    document.getElementById('attDepartmentFilter').value = 'all';
    document.getElementById('attStatusFilter').value = 'all';
    setDefaultDateRange();
    filterAttendance();
}

function updateAttendanceStats() {
    const attendance = getFilteredAttendance();
    
    let present = 0, late = 0, absent = 0;
    
    attendance.forEach(record => {
        if (record.status === 0) present++;
        else if (record.status === 2) late++;
        else if (record.status === 1) absent++;
    });
    
    const total = present + late + absent;
    const percentage = total > 0 ? ((present + late) / total * 100).toFixed(1) : 0;
    
    document.getElementById('attPresent').textContent = present;
    document.getElementById('attLate').textContent = late;
    document.getElementById('attAbsent').textContent = absent;
    document.getElementById('attPercentage').textContent = percentage + '%';
}

async function exportAttendance() {
    try {
        const attendance = getFilteredAttendance();
        
        if (attendance.length === 0) {
            showToast('No data to export', 'warning');
            return;
        }
        
        let csv = 'Date,Time,Name,Roll Number,Department,Subject,Status,Method\n';
        
        attendance.forEach(record => {
            const user = systemData.users.find(u => u.user_id === record.user_id);
            if (!user) return;
            
            const date = new Date(record.timestamp * 1000);
            const dateStr = date.toLocaleDateString('en-US');
            const timeStr = date.toLocaleTimeString('en-US');
            
            csv += `${dateStr},${timeStr},${user.name},${user.roll_number || ''},`;
            csv += `${user.department || ''},${record.session_name || ''},`;
            csv += `${getStatusLabel(record.status)},${record.auth_method === 0 ? 'Fingerprint' : 'RFID'}\n`;
        });
        
        downloadCSV(csv, 'attendance_report.csv');
        showToast('Report exported successfully', 'success');
        
    } catch (error) {
        console.error('Export error:', error);
        showToast('Export failed', 'error');
    }
}

// ========================================
// Session Management
// ========================================

async function loadSessionControl() {
    const container = document.getElementById('sessionControl');
    const session = systemData.current_session;
    
    if (session && session.active) {
        const startTime = new Date(session.start_time * 1000);
        const duration = Math.floor((Date.now() / 1000 - session.start_time) / 60);
        
        container.innerHTML = `
            <div class="session-active">
                <h2 style="color: var(--success-color); margin-bottom: 1.5rem;">
                    ✓ Session Active
                </h2>
                <div class="session-info">
                    <div class="session-info-item">
                        <span class="session-info-label">Subject:</span>
                        <span class="session-info-value">${escapeHtml(session.subject_name)}</span>
                    </div>
                    <div class="session-info-item">
                        <span class="session-info-label">Faculty:</span>
                        <span class="session-info-value">${escapeHtml(session.faculty_name)}</span>
                    </div>
                    <div class="session-info-item">
                        <span class="session-info-label">Department:</span>
                        <span class="session-info-value">${escapeHtml(session.department || 'All')}</span>
                    </div>
                    <div class="session-info-item">
                        <span class="session-info-label">Duration:</span>
                        <span class="session-info-value">${duration} minutes</span>
                    </div>
                    <div class="session-info-item">
                        <span class="session-info-label">Present:</span>
                        <span class="session-info-value" style="color: var(--success-color); font-weight: 700;">
                            ${session.total_present || 0}
                        </span>
                    </div>
                    <div class="session-info-item">
                        <span class="session-info-label">Absent:</span>
                        <span class="session-info-value" style="color: var(--danger-color); font-weight: 700;">
                            ${session.total_absent || 0}
                        </span>
                    </div>
                </div>
                <button class="btn btn-danger btn-full" style="margin-top: 1.5rem;" onclick="stopSession()">
                    Stop Session
                </button>
            </div>
        `;
    } else {
        container.innerHTML = `
            <div class="session-inactive">
                <h2 style="margin-bottom: 1rem;">No Active Session</h2>
                <p style="margin-bottom: 1.5rem;">Start a new session to begin taking attendance</p>
                <button class="btn btn-primary btn-full" onclick="showSessionModal()">
                    Start New Session
                </button>
            </div>
        `;
    }
    
    await loadSessionHistory();
}

function showSessionModal() {
    document.getElementById('sessionForm').reset();
    document.getElementById('sessionModal').classList.add('active');
}

function closeSessionModal() {
    document.getElementById('sessionModal').classList.remove('active');
}

async function startSession(subject, faculty, department) {
    try {
        const result = await apiCall('/api/session/start', 'POST', {
            subject: subject,
            faculty: faculty,
            department: department
        });
        
        if (result && result.success) {
            systemData.current_session = {
                active: true,
                subject_name: subject,
                faculty_name: faculty,
                department: department,
                start_time: Math.floor(Date.now() / 1000),
                total_present: 0,
                total_absent: 0
            };
            
            showToast('Session started successfully!', 'success');
            closeSessionModal();
            await loadSessionControl();
            
            if (currentPage === 'dashboard') {
                await loadSessionStatus();
            }
        } else {
            showToast('Failed to start session', 'error');
        }
    } catch (error) {
        console.error('Start session error:', error);
        showToast('Error starting session', 'error');
    }
}

async function stopSession() {
    if (!confirm('Are you sure you want to stop the current session?')) {
        return;
    }
    
    try {
        const result = await apiCall('/api/session/stop', 'POST');
        
        if (result && result.success) {
            if (systemData.current_session) {
                systemData.current_session.active = false;
                systemData.current_session.end_time = Math.floor(Date.now() / 1000);
            }
            
            showToast('Session stopped', 'success');
            await loadSessionControl();
            
            if (currentPage === 'dashboard') {
                await loadSessionStatus();
            }
        } else {
            showToast('Failed to stop session', 'error');
        }
    } catch (error) {
        console.error('Stop session error:', error);
        showToast('Error stopping session', 'error');
    }
}

async function loadSessionHistory() {
    const tbody = document.getElementById('sessionHistoryBody');
    const sessions = systemData.sessions || [];
    
    tbody.innerHTML = '';
    
    if (sessions.length === 0) {
        tbody.innerHTML = `
            <tr>
                <td colspan="7" class="text-center">
                    <div class="empty-state">
                        <div class="empty-state-icon">🎯</div>
                        <div class="empty-state-text">No session history</div>
                    </div>
                </td>
            </tr>
        `;
        return;
    }
    
    sessions.slice(-20).reverse().forEach(session => {
        const date = new Date(session.start_time * 1000).toLocaleDateString('en-US');
        const duration = session.end_time ? 
            Math.floor((session.end_time - session.start_time) / 60) : 
            'Ongoing';
        
        const row = `
            <tr>
                <td>${date}</td>
                <td>${escapeHtml(session.subject_name)}</td>
                <td>${escapeHtml(session.faculty_name)}</td>
                <td>${duration} ${duration !== 'Ongoing' ? 'min' : ''}</td>
                <td style="color: var(--success-color); font-weight: 600;">${session.total_present || 0}</td>
                <td style="color: var(--danger-color); font-weight: 600;">${session.total_absent || 0}</td>
                <td>
                    <button class="action-btn action-btn-view" onclick="viewSessionDetails('${session.session_id}')">
                        👁️ View
                    </button>
                </td>
            </tr>
        `;
        
        tbody.innerHTML += row;
    });
}

function viewSessionDetails(sessionId) {
    showToast('Viewing session details: ' + sessionId, 'info');
    // Implement detailed session view
}

// ========================================
// Settings Management
// ========================================

async function loadSettings() {
    const config = systemData.config || {};
    
    // Load classroom settings
    document.getElementById('classroomId').value = config.classroom_id || 'CS-101';
    document.getElementById('classroomName').value = config.classroom_name || 'Computer Lab';
    document.getElementById('settingsDepartment').value = config.department || 'Computer Science';
    
    // Load auth settings
    document.getElementById('authMode').value = config.auth_mode || 'both';
    document.getElementById('enableFingerprint').checked = config.fingerprint_enabled !== false;
    document.getElementById('enableRFID').checked = config.rfid_enabled !== false;
    document.getElementById('enableBuzzer').checked = config.buzzer_enabled !== false;
    
    // Load rules
    document.getElementById('lateThreshold').value = config.late_threshold || 15;
    document.getElementById('allowMultipleEntry').checked = config.allow_multiple_entry || false;
    document.getElementById('autoMarkAbsent').checked = config.auto_mark_absent !== false;
}

async function saveClassroomSettings() {
    const config = {
        classroom_id: document.getElementById('classroomId').value,
        classroom_name: document.getElementById('classroomName').value,
        department: document.getElementById('settingsDepartment').value
    };
    
    try {
        const result = await apiCall('/api/settings/classroom', 'POST', config);
        
        if (result && result.success) {
            systemData.config = { ...systemData.config, ...config };
            showToast('Classroom settings saved', 'success');
        } else {
            showToast('Failed to save settings', 'error');
        }
    } catch (error) {
        console.error('Save settings error:', error);
        showToast('Error saving settings', 'error');
    }
}

async function saveAuthSettings() {
    const config = {
        auth_mode: document.getElementById('authMode').value,
        fingerprint_enabled: document.getElementById('enableFingerprint').checked,
        rfid_enabled: document.getElementById('enableRFID').checked,
        buzzer_enabled: document.getElementById('enableBuzzer').checked
    };
    
    try {
        const result = await apiCall('/api/settings/auth', 'POST', config);
        
        if (result && result.success) {
            systemData.config = { ...systemData.config, ...config };
            showToast('Authentication settings saved', 'success');
        } else {
            showToast('Failed to save settings', 'error');
        }
    } catch (error) {
        console.error('Save auth settings error:', error);
        showToast('Error saving settings', 'error');
    }
}

async function saveRulesSettings() {
    const config = {
        late_threshold: parseInt(document.getElementById('lateThreshold').value),
        allow_multiple_entry: document.getElementById('allowMultipleEntry').checked,
        auto_mark_absent: document.getElementById('autoMarkAbsent').checked
    };
    
    try {
        const result = await apiCall('/api/settings/rules', 'POST', config);
        
        if (result && result.success) {
            systemData.config = { ...systemData.config, ...config };
            showToast('Rules saved successfully', 'success');
        } else {
            showToast('Failed to save rules', 'error');
        }
    } catch (error) {
        console.error('Save rules error:', error);
        showToast('Error saving rules', 'error');
    }
}

async function backupDatabase() {
    try {
        showToast('Creating backup...', 'info');
        
        const backup = {
            timestamp: Date.now(),
            version: '1.0',
            users: systemData.users,
            attendance: systemData.attendance,
            sessions: systemData.sessions,
            config: systemData.config
        };
        
        const json = JSON.stringify(backup, null, 2);
        const blob = new Blob([json], { type: 'application/json' });
        const url = URL.createObjectURL(blob);
        
        const a = document.createElement('a');
        a.href = url;
        a.download = `attendance_backup_${Date.now()}.json`;
        a.click();
        
        URL.revokeObjectURL(url);
        
        showToast('Backup created successfully', 'success');
    } catch (error) {
        console.error('Backup error:', error);
        showToast('Backup failed', 'error');
    }
}

function restoreDatabase() {
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = '.json';
    
    input.onchange = async (e) => {
        const file = e.target.files[0];
        if (!file) return;
        
        try {
            const text = await file.text();
            const backup = JSON.parse(text);
            
            if (confirm('This will replace all current data. Continue?')) {
                const result = await apiCall('/api/database/restore', 'POST', backup);
                
                if (result && result.success) {
                    systemData = backup;
                    showToast('Database restored successfully', 'success');
                    await loadSystemData();
                    location.reload();
                } else {
                    showToast('Restore failed', 'error');
                }
            }
        } catch (error) {
            console.error('Restore error:', error);
            showToast('Invalid backup file', 'error');
        }
    };
    
    input.click();
}

async function clearAttendanceRecords() {
    if (!confirm('This will delete ALL attendance records. This action cannot be undone. Continue?')) {
        return;
    }
    
    if (!confirm('Are you ABSOLUTELY sure? Type "DELETE" to confirm.')) {
        return;
    }
    
    try {
        const result = await apiCall('/api/database/clear-attendance', 'GET');
        
        if (result && result.success) {
            systemData.attendance = [];
            showToast('Attendance records cleared', 'success');
            await loadSystemData();
        } else {
            showToast('Failed to clear records', 'error');
        }
    } catch (error) {
        console.error('Clear records error:', error);
        showToast('Error clearing records', 'error');
    }
}

async function resetSystem() {
    if (!confirm('WARNING: This will delete ALL data and reset the system. Continue?')) {
        return;
    }
    
    const confirmation = prompt('Type "RESET" to confirm:');
    if (confirmation !== 'RESET') {
        showToast('Reset cancelled', 'info');
        return;
    }
    
    try {
        const result = await apiCall('/api/system/reset', 'GET');
        
        if (result && result.success) {
            showToast('System reset complete. Restarting...', 'success');
            setTimeout(() => {
                location.reload();
            }, 2000);
        } else {
            showToast('Reset failed', 'error');
        }
    } catch (error) {
        console.error('Reset error:', error);
        showToast('Reset error', 'error');
    }
}

// ========================================
// Form Handlers
// ========================================

function setupFormHandlers() {
    // User form
    document.getElementById('userForm').addEventListener('submit', async function(e) {
        e.preventDefault();
        await saveUser();
    });
    
    // Session form
    document.getElementById('sessionForm').addEventListener('submit', async function(e) {
        e.preventDefault();
        
        const subject = document.getElementById('sessionSubject').value;
        const faculty = document.getElementById('sessionFaculty').value;
        const department = document.getElementById('sessionDepartment').value;
        
        await startSession(subject, faculty, department);
    });
    
    // Close modals on background click
    document.querySelectorAll('.modal').forEach(modal => {
        modal.addEventListener('click', function(e) {
            if (e.target === modal) {
                modal.classList.remove('active');
            }
        });
    });
}

async function saveUser() {

    const userData = {
        name: document.getElementById('userName').value,
        type: getUserTypeFromValue(document.getElementById('userType').value),
        roll_number: document.getElementById('userRollNumber').value,
        email: document.getElementById('userEmail').value,
        department: document.getElementById('userDepartment').value,
        semester: parseInt(document.getElementById('userSemester').value),
        status: getUserStatusFromValue(document.getElementById('userStatus').value)
    };
    
    if (!userData.name) {
        showToast('Please enter user name', 'warning');
        return;
    }
    
    try {
        const endpoint = currentEditUserId !== null ? 
            '/api/user/update' : '/api/user/add';
        
        if (currentEditUserId !== null) {
            userData.user_id = currentEditUserId;
        }
        
        const result = await apiCall(endpoint, 'POST', userData);
        
        if (result && result.success) {
            if (currentEditUserId !== null) {
                // Update existing user
                const index = systemData.users.findIndex(u => u.user_id === currentEditUserId);
                if (index !== -1) {
                    systemData.users[index] = { ...systemData.users[index], ...userData };
                }
                showToast('User updated successfully', 'success');
            } else {
                // Add new user
                userData.user_id = result.user_id || systemData.users.length;
                systemData.users.push(userData);
                showToast('User added successfully', 'success');
            }
            
            closeUserModal();
            await displayUsers();
            await updateDashboardStats();
        } else {
            showToast('Failed to save user', result.status.message);
        }
    } catch (error) {
        console.error('Save user error:', error);
        showToast('Error saving user', 'error');
    }
}

// ========================================
// Utility Functions
// ========================================

function escapeHtml(text) {
    if (!text) return '';
    const div = document.createElement('div');
    div.textContent = text;
    return div.innerHTML;
}

function getUserTypeLabel(type) {
    const types = {
        0: '<span class="badge badge-success">Student</span>',
        1: '<span class="badge badge-info">Faculty</span>',
        2: '<span class="badge badge-warning">Admin</span>',
        3: '<span class="badge">Guest</span>'
    };
    return types[type] || 'Unknown';
}

function getUserTypeValue(type) {
    const values = {
        0: 'student',
        1: 'faculty',
        2: 'admin',
        3: 'guest'
    };
    return values[type] || 'student';
}

function getUserTypeFromValue(value) {
    const types = {
        'student': 0,
        'faculty': 1,
        'admin': 2,
        'guest': 3
    };
    return types[value] || 0;
}

function getUserStatusValue(status) {
    const values = {
        0: 'active',
        1: 'inactive',
        2: 'suspended',
        3: 'graduated'
    };
    return values[status] || 'active';
}

function getUserStatusFromValue(value) {
    const statuses = {
        'active': 0,
        'inactive': 1,
        'suspended': 2,
        'graduated': 3
    };
    return statuses[value] || 0;
}

function getStatusLabel(status) {
    const labels = {
        0: 'Present',
        1: 'Absent',
        2: 'Late',
        3: 'Leave'
    };
    return labels[status] || 'Unknown';
}

function getStatusClass(status) {
    const classes = {
        0: 'status-present',
        1: 'status-absent',
        2: 'status-late',
        3: 'status-absent'
    };
    return classes[status] || '';
}

function getStatusValue(statusName) {
    const values = {
        'present': 0,
        'absent': 1,
        'late': 2,
        'leave': 3
    };
    return values[statusName] || 0;
}

function calculateAttendancePercentage(user) {
    if (!user.total_classes || user.total_classes === 0) {
        return 0;
    }
    return ((user.total_attendance / user.total_classes) * 100).toFixed(1);
}

async function populateDepartmentFilters() {
    const departments = [...new Set(systemData.users
        .map(u => u.department)
        .filter(d => d))];
    
    const filterSelects = [
        document.getElementById('departmentFilter'),
        document.getElementById('attDepartmentFilter')
    ];
    
    filterSelects.forEach(select => {
        if (!select) return;
        
        // Keep "All" option
        const allOption = select.querySelector('option[value="all"]');
        select.innerHTML = '';
        if (allOption) {
            select.appendChild(allOption);
        }
        
        // Add department options
        departments.forEach(dept => {
            const option = document.createElement('option');
            option.value = dept;
            option.textContent = dept;
            select.appendChild(option);
        });
    });
}

async function populateAttendanceDepartmentFilter() {
    await populateDepartmentFilters();
}

function downloadCSV(csvContent, filename) {
    const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
    const link = document.createElement('a');
    
    if (navigator.msSaveBlob) {
        navigator.msSaveBlob(blob, filename);
    } else {
        link.href = URL.createObjectURL(blob);
        link.download = filename;
        link.style.display = 'none';
        document.body.appendChild(link);
        link.click();
        document.body.removeChild(link);
    }
}

async function exportUsers() {
    try {
        const users = systemData.users;
        
        if (users.length === 0) {
            showToast('No users to export', 'warning');
            return;
        }
        
        let csv = 'ID,Name,Type,Roll Number,Department,Email,Semester,Fingerprint,RFID,Attendance %,Status\n';
        
        users.forEach(user => {
            const attendance = calculateAttendancePercentage(user);
            csv += `${user.user_id},${user.name},${getUserTypeLabel(user.type).replace(/<[^>]*>/g, '')},`;
            csv += `${user.roll_number || ''},${user.department || ''},${user.email || ''},`;
            csv += `${user.semester || ''},${user.fingerprint_enrolled ? 'Yes' : 'No'},`;
            csv += `${user.rfid_enrolled ? 'Yes' : 'No'},${attendance}%,`;
            csv += `${getUserStatusValue(user.status)}\n`;
        });
        
        downloadCSV(csv, 'users_export.csv');
        showToast('Users exported successfully', 'success');
        
    } catch (error) {
        console.error('Export error:', error);
        showToast('Export failed', 'error');
    }
}

function printUsers() {
    window.print();
}

function updateTimeDisplay() {
    const now = new Date();
    const timeStr = now.toLocaleTimeString('en-US', {
        hour: '2-digit',
        minute: '2-digit',
        second: '2-digit',
        hour12: false
    });
    
    const timeDisplay = document.getElementById('timeDisplay');
    if (timeDisplay) {
        timeDisplay.textContent = timeStr;
    }
}

function showToast(message, type = 'info') {
    const toast = document.getElementById('toast');
    
    toast.textContent = message;
    toast.className = 'toast show ' + type;
    
    setTimeout(() => {
        toast.classList.remove('show');
    }, 3000);
}

// ========================================
// Mock Data for Testing (Development Only)
// ========================================

function generateMockData() {
    // Mock users
    systemData.users = [
        {
            user_id: 0,
            name: 'John Doe',
            type: 0,
            roll_number: 'CS2021001',
            email: 'john@university.edu',
            department: 'Computer Science',
            semester: 6,
            status: 0,
            fingerprint_enrolled: true,
            rfid_enrolled: true,
            total_attendance: 45,
            total_classes: 50,
            created_at: Math.floor(Date.now() / 1000) - 86400 * 30,
            last_seen: Math.floor(Date.now() / 1000) - 3600
        },
        {
            user_id: 1,
            name: 'Jane Smith',
            type: 0,
            roll_number: 'CS2021002',
            email: 'jane@university.edu',
            department: 'Computer Science',
            semester: 6,
            status: 0,
            fingerprint_enrolled: true,
            rfid_enrolled: false,
            total_attendance: 48,
            total_classes: 50,
            created_at: Math.floor(Date.now() / 1000) - 86400 * 30,
            last_seen: Math.floor(Date.now() / 1000) - 7200
        },
        {
            user_id: 2,
            name: 'Prof. Robert Johnson',
            type: 1,
            roll_number: 'FAC001',
            email: 'robert@university.edu',
            department: 'Computer Science',
            semester: 0,
            status: 0,
            fingerprint_enrolled: true,
            rfid_enrolled: true,
            total_attendance: 0,
            total_classes: 0,
            created_at: Math.floor(Date.now() / 1000) - 86400 * 60,
            last_seen: Math.floor(Date.now() / 1000) - 1800
        }
    ];
    
    // Mock attendance
    systemData.attendance = [];
    const now = Math.floor(Date.now() / 1000);
    
    for (let i = 0; i < 20; i++) {
        const userId = i % 3;
        systemData.attendance.push({
            record_id: i,
            user_id: userId,
            timestamp: now - (i * 3600),
            status: i % 5 === 0 ? 2 : 0,
            auth_method: i % 2,
            session_name: 'Computer Networks',
            session_id: 'SES001'
        });
    }
    
    // Mock session
    systemData.current_session = {
        active: true,
        session_id: 'SES001',
        subject_name: 'Computer Networks',
        faculty_name: 'Prof. Robert Johnson',
        department: 'Computer Science',
        start_time: now - 3600,
        total_present: 45,
        total_absent: 5
    };
    
    // Mock sessions history
    systemData.sessions = [
        {
            session_id: 'SES001',
            subject_name: 'Computer Networks',
            faculty_name: 'Prof. Robert Johnson',
            start_time: now - 86400,
            end_time: now - 86400 + 3600,
            total_present: 42,
            total_absent: 8
        }
    ];
    
    // Mock stats
    systemData.stats = {
        total_users: 3,
        total_students: 2,
        total_faculty: 1,
        today_attendance: 45,
        total_attendance_records: 20
    };
    
    // Mock config
    systemData.config = {
        classroom_id: 'CS-101',
        classroom_name: 'Computer Lab',
        department: 'Computer Science',
        auth_mode: 'both',
        fingerprint_enabled: true,
        rfid_enabled: true,
        buzzer_enabled: true,
        late_threshold: 15,
        allow_multiple_entry: false,
        auto_mark_absent: true
    };
}

// Uncomment for development/testing
// generateMockData();

console.log('Smart Attendance System loaded successfully');